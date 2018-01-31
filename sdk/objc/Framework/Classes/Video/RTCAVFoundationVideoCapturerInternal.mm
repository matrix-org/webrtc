/*
 *  Copyright 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#import "RTCAVFoundationVideoCapturerInternal.h"

#import <Foundation/Foundation.h>
#if TARGET_OS_IPHONE
#import <UIKit/UIKit.h>
#import "WebRTC/UIDevice+RTCDevice.h"
#endif

#import "AVCaptureSession+DevicePosition.h"
#import "RTCDispatcher+Private.h"
#import "WebRTC/RTCLogging.h"

#include "avfoundationformatmapper.h"

@implementation RTCAVFoundationVideoCapturerInternal {
  // Keep pointers to inputs for convenience.
  AVCaptureDeviceInput *_frontCameraInput;
  AVCaptureDeviceInput *_backCameraInput;
  AVCaptureVideoDataOutput *_videoDataOutput;
  AVCaptureDepthDataOutput *_depthDataOutput;
  // The cricket::VideoCapturer that owns this class. Should never be NULL.
  webrtc::AVFoundationVideoCapturer *_capturer;
  BOOL _hasRetriedOnFatalError;
  BOOL _isRunning;
  BOOL _hasStarted;
  rtc::CriticalSection _crit;
#if TARGET_OS_IPHONE
  UIDeviceOrientation _orientation;
#endif
}

// Static look-up tables to convert 16-bit depth into YUV
namespace {
    uint8_t depthToY[65536];
    uint8_t depthToQuarterU[65536];
    uint8_t depthToQuarterV[65536];
}

@synthesize captureSession = _captureSession;
@synthesize frameQueue = _frameQueue;
@synthesize useBackCamera = _useBackCamera;

@synthesize isRunning = _isRunning;
@synthesize hasStarted = _hasStarted;

// converts 8-bit RGB components into 8-bit YUV.
// Chrominance is added at quarter-amplitude to the existing UV values in order to
// facilitate 4:2:0 downsampling.
inline void rgb2yuv(uint8_t r, uint8_t g, uint8_t b, uint8_t * y, uint8_t * u, uint8_t * v) {
  // Using formulas from http://msdn.microsoft.com/en-us/library/ms893078
  *y =      ((66 * r  + 129 * g + 25  * b + 128) >> 8) + 16;
  // N.B. this factors in a divide-by-4 to let you downsample chroma as you go...
  *u = *u + ((-38 * r - 74  * g + 112 * b + 128) >> 10) + 32;
  *v = *v + ((112 * r - 94  * g - 18  * b + 128) >> 10) + 32;
}

+ (void) initialize {
  // implement depth to RGB as per http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf
  // we also convert to 420p as that's what WebRTC insists on

  // periodicity constants as per the paper (end of sec 3)
  double np = 512.0;
  double w = 65536.0;
  double p = np / w;

  uint8_t * dstY = depthToY;
  uint8_t * dstQuarterU = depthToQuarterU;
  uint8_t * dstQuarterV = depthToQuarterV;

  // assuming truedepth camera is giving us IEEE 754-2008 half-precision 16-bit floats, this means
  // that positives lie between 0.0 through 65504.0, which when cast to a uint16_t lie between 0 and 65403

  // build our depth->YUV LUT
  for (size_t d = 0; d < 65536; d++) {
    // the paper describes three colour components: L, Ha and Hb, which we map to BGR.
    // L is low-res depth data; H is high-res.

    double L = (d + 0.5) / w;

    double Ha = fmod(L / (p / 2.0), 2.0);
    if (Ha > 1.0) Ha = 2.0 - Ha;

    // we add 1.0 to avoid taking the modulus of a negative number
    double Hb = fmod((1.0 + L - (p / 4.0)) / (p / 2.0), 2.0);
    if (Hb > 1.0) Hb = 2.0 - Hb;

    // rescale L in order to increase its dynamic range, as in practice the data
    // we get from the truedepth camera seems to only be between 10K and 20K, rather
    // than the 0K-65K range we're considering here...
    // L *= 4.0;
    // L -= 0.3;

    L = 1.0 - (L*8.0); // increase contrast by 8x to make it easier to encode.
    // for now, ditch the delta encoding as the YUV colourspace conversion seems
    // to make it unusable.
    Ha = L;
    Hb = L; 

    rgb2yuv(Hb * 255, Ha * 255, L * 255, dstY, dstQuarterU, dstQuarterV);

    dstY++;
    dstQuarterU++;
    dstQuarterV++;
  }
}

// This is called from the thread that creates the video source, which is likely
// the main thread.
- (instancetype)initWithCapturer:(webrtc::AVFoundationVideoCapturer *)capturer {
  RTC_DCHECK(capturer);
  if (self = [super init]) {
    _capturer = capturer;
    // Create the capture session and all relevant inputs and outputs. We need
    // to do this in init because the application may want the capture session
    // before we start the capturer for e.g. AVCapturePreviewLayer. All objects
    // created here are retained until dealloc and never recreated.
    if (![self setupCaptureSession]) {
      return nil;
    }
    NSNotificationCenter *center = [NSNotificationCenter defaultCenter];
#if TARGET_OS_IPHONE
    _orientation = UIDeviceOrientationPortrait;
    [center addObserver:self
               selector:@selector(deviceOrientationDidChange:)
                   name:UIDeviceOrientationDidChangeNotification
                 object:nil];
    [center addObserver:self
               selector:@selector(handleCaptureSessionInterruption:)
                   name:AVCaptureSessionWasInterruptedNotification
                 object:_captureSession];
    [center addObserver:self
               selector:@selector(handleCaptureSessionInterruptionEnded:)
                   name:AVCaptureSessionInterruptionEndedNotification
                 object:_captureSession];
    [center addObserver:self
               selector:@selector(handleApplicationDidBecomeActive:)
                   name:UIApplicationDidBecomeActiveNotification
                 object:[UIApplication sharedApplication]];
#endif
    [center addObserver:self
               selector:@selector(handleCaptureSessionRuntimeError:)
                   name:AVCaptureSessionRuntimeErrorNotification
                 object:_captureSession];
    [center addObserver:self
               selector:@selector(handleCaptureSessionDidStartRunning:)
                   name:AVCaptureSessionDidStartRunningNotification
                 object:_captureSession];
    [center addObserver:self
               selector:@selector(handleCaptureSessionDidStopRunning:)
                   name:AVCaptureSessionDidStopRunningNotification
                 object:_captureSession];
  }
  return self;
}

- (void)dealloc {
  RTC_DCHECK(!self.hasStarted);
  [[NSNotificationCenter defaultCenter] removeObserver:self];
  _capturer = nullptr;
}

- (AVCaptureSession *)captureSession {
  return _captureSession;
}

- (AVCaptureDevice *)getActiveCaptureDevice {
  return self.useBackCamera ? _backCameraInput.device : _frontCameraInput.device;
}

- (nullable AVCaptureDevice *)frontCaptureDevice {
  return _frontCameraInput.device;
}

- (nullable AVCaptureDevice *)backCaptureDevice {
  return _backCameraInput.device;
}

- (dispatch_queue_t)frameQueue {
  if (!_frameQueue) {
    _frameQueue =
        dispatch_queue_create("org.webrtc.avfoundationvideocapturer.video", DISPATCH_QUEUE_SERIAL);
    dispatch_set_target_queue(_frameQueue,
                              dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0));
  }
  return _frameQueue;
}

// Called from any thread (likely main thread).
- (BOOL)canUseBackCamera {
  return _backCameraInput != nil;
}

// Called from any thread (likely main thread).
- (BOOL)useBackCamera {
  @synchronized(self) {
    return _useBackCamera;
  }
}

// Called from any thread (likely main thread).
- (void)setUseBackCamera:(BOOL)useBackCamera {
  if (!self.canUseBackCamera) {
    if (useBackCamera) {
      RTCLogWarning(@"No rear-facing camera exists or it cannot be used;"
                     "not switching.");
    }
    return;
  }
  @synchronized(self) {
    if (_useBackCamera == useBackCamera) {
      return;
    }
    _useBackCamera = useBackCamera;
    [self updateSessionInputForUseBackCamera:useBackCamera];
  }
}

// Called from WebRTC thread.
- (void)start {
  if (self.hasStarted) {
    return;
  }
  self.hasStarted = YES;
  [RTCDispatcher
      dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                    block:^{
                        [self updateOrientation];
#if TARGET_OS_IPHONE
                        [[UIDevice currentDevice] beginGeneratingDeviceOrientationNotifications];
#endif
                        AVCaptureSession *captureSession = self.captureSession;
                        [captureSession startRunning];
                    }];
}

// Called from same thread as start.
- (void)stop {
  if (!self.hasStarted) {
    return;
  }
  self.hasStarted = NO;
  // Due to this async block, it's possible that the ObjC object outlives the
  // C++ one. In order to not invoke functions on the C++ object, we set
  // hasStarted immediately instead of dispatching it async.
  [RTCDispatcher
      dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                    block:^{
                        if (_videoDataOutput) {
                          [_videoDataOutput setSampleBufferDelegate:nil queue:nullptr];
                        }
                        // if (_depthDataOutput) {
                        //   [_depthDataOutput setDelegate:nil callbackQueue:nullptr];
                        // }
                        [_captureSession stopRunning];
#if TARGET_OS_IPHONE
                        [[UIDevice currentDevice] endGeneratingDeviceOrientationNotifications];
#endif
                    }];
}

#pragma mark iOS notifications

#if TARGET_OS_IPHONE
- (void)deviceOrientationDidChange:(NSNotification *)notification {
  [RTCDispatcher dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                               block:^{
                                   [self updateOrientation];
                               }];
}
#endif

#pragma mark AVCaptureVideoDataOutputSampleBufferDelegate

- (void)captureOutput:(AVCaptureOutput *)captureOutput
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
           fromConnection:(AVCaptureConnection *)connection {
  NSParameterAssert(captureOutput == _videoDataOutput);
  if (!self.hasStarted) {
    return;
  }

#if TARGET_OS_IPHONE
  // Default to portrait orientation on iPhone.
  webrtc::VideoRotation rotation = webrtc::kVideoRotation_90;
  BOOL usingFrontCamera = NO;
  // Check the image's EXIF for the camera the image came from as the image could have been
  // delayed as we set alwaysDiscardsLateVideoFrames to NO.
  AVCaptureDevicePosition cameraPosition =
      [AVCaptureSession devicePositionForSampleBuffer:sampleBuffer];
  if (cameraPosition != AVCaptureDevicePositionUnspecified) {
    usingFrontCamera = AVCaptureDevicePositionFront == cameraPosition;
  } else {
    AVCaptureDeviceInput *deviceInput =
        (AVCaptureDeviceInput *)((AVCaptureInputPort *)connection.inputPorts.firstObject).input;
    usingFrontCamera = AVCaptureDevicePositionFront == deviceInput.device.position;
  }
  switch (_orientation) {
    case UIDeviceOrientationPortrait:
      rotation = webrtc::kVideoRotation_90;
      break;
    case UIDeviceOrientationPortraitUpsideDown:
      rotation = webrtc::kVideoRotation_270;
      break;
    case UIDeviceOrientationLandscapeLeft:
      rotation = usingFrontCamera ? webrtc::kVideoRotation_180 : webrtc::kVideoRotation_0;
      break;
    case UIDeviceOrientationLandscapeRight:
      rotation = usingFrontCamera ? webrtc::kVideoRotation_0 : webrtc::kVideoRotation_180;
      break;
    case UIDeviceOrientationFaceUp:
    case UIDeviceOrientationFaceDown:
    case UIDeviceOrientationUnknown:
      // Ignore.
      break;
  }
#else
  // No rotation on Mac.
  webrtc::VideoRotation rotation = webrtc::kVideoRotation_0;
#endif

  _capturer->CaptureSampleBuffer(sampleBuffer, rotation);
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
    didDropSampleBuffer:(CMSampleBufferRef)sampleBuffer
         fromConnection:(AVCaptureConnection *)connection {
  RTCLogError(@"Dropped sample buffer.");
}

#pragma mark AVCaptureDepthDataOutputDelegate

inline float half2float(uint16_t d) {
  // stolen from https://stackoverflow.com/a/26779139/6764493
  // XXX: doesn't handles subnormals, infinities and other edge cases
  // but hopefully the truedepth camera doesn't hand us any... :S
  uint32_t out = ((((uint32_t)d & 0x8000) << 16) | 
                 ((((uint32_t)d & 0x7c00) + 0x1C000) << 13) | 
                  (((uint32_t)d & 0x03FF) << 13) );
  return *(float *)&out;
}

- (void)depthDataOutput:(AVCaptureDepthDataOutput *)depthDataOutput
     didOutputDepthData:(AVDepthData *)depthData
              timestamp:(CMTime)timestamp
             connection:(AVCaptureConnection *)connection {

  NSParameterAssert(depthDataOutput == _depthDataOutput);
  if (!self.hasStarted) {
    return;
  }

  // Assume that rotation metadata, if any, will be tracked by non-depth capture
  // where we can use AVCaptureSession devicePositionForSampleBuffer correctly
  webrtc::VideoRotation rotation = webrtc::kVideoRotation_90;

  // Convert our depthData from disparity into depth...
  // unsigned int t = depthData.depthDataType;
  // RTCLogInfo(@"depthDataType is %.*s", 4, (char *)&t);
  if (depthData.depthDataType != kCVPixelFormatType_DepthFloat16) {
    depthData = [depthData depthDataByConvertingToDepthDataType:kCVPixelFormatType_DepthFloat16];
  }

  OSStatus status;
  CMFormatDescriptionRef desc = NULL;
  status = CMVideoFormatDescriptionCreateForImageBuffer(NULL, depthData.depthDataMap, &desc);
  if (status != noErr) {
    RTCLogError(@"CMVideoFormatDescriptionCreateForImageBuffer failed to set: %d", status);
  }

  CMSampleTimingInfo timing;
  timing.duration = kCMTimeInvalid;
  timing.presentationTimeStamp = timestamp;
  timing.decodeTimeStamp = kCMTimeInvalid;  

  // Convert our depthData into a SampleBuffer
  CVImageBufferRef imageBuffer;
  CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(desc);

  // FIXME: use a CVPixelBufferPool for efficiency
  status = CVPixelBufferCreate(
    kCFAllocatorDefault,
    dims.width,
    dims.height,
    kCVPixelFormatType_420YpCbCr8BiPlanarFullRange,
    nil,
    &imageBuffer
  );
  if (status != noErr) {
    RTCLogError(@"CVPixelBufferCreate failed: %d", status);
  }

  status = CVPixelBufferLockBaseAddress(depthData.depthDataMap, 0);
  if (status != noErr) {
    RTCLogError(@"CVPixelBufferLockBaseAddress failed: %d", status);
  }
  status = CVPixelBufferLockBaseAddress(imageBuffer, 0);
  if (status != noErr) {
    RTCLogError(@"CVPixelBufferLockBaseAddress failed: %d", status);
  }

  // uint16_t min = 65535;
  // uint16_t max = 0;

  uint16_t * src = (uint16_t *) CVPixelBufferGetBaseAddress(depthData.depthDataMap);
  uint8_t * dstY = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 0);
  uint8_t * dstC = (uint8_t *) CVPixelBufferGetBaseAddressOfPlane(imageBuffer, 1);
  for (int y = 0; y < dims.height; y++) {
    for (int x = 0; x < dims.width; x++) {

      // if (y == 100) RTCLogInfo(@"%d, %d = %d", x, y, *src);
      // if (*src < min) min = *src;
      // if (*src > max) max = *src;
      
      // we don't cast the half to a uint16_t, as the dynamic range ends up massively
      // compressed and we have no way to expand it out again on the rendering side
      // (short of bit twiddling to do half->float in GLSL ES 3.0, which requires
      // WebGL 2.0, which aframe doesn't easily support yet).

      // Let's assume we want to capture everything within 8 metres of the phone...
      // but dumping everything further than 1m as background noise.  We don't just scale
      // everything to fit in 1m as the banding gets too bad. Yes, this does deliberately
      // reduce our encoded depth resolution.
      size_t val = 65535 * (half2float(*src) / 8.0);
      if (val > 65535/8) { val = 65535; }

      *dstY = depthToY[val];
      *dstC = *dstC + depthToQuarterU[val];
      dstC++;      
      *dstC = *dstC + depthToQuarterV[val];

      src++;
      dstY++;
      if ((x & 1) == 0) {
        dstC++; // only advance chroma every other X pixel
      } else {
        dstC--;
      }
    }
    if ((y & 1) == 0) { // only advance chroma every other Y pixel
      dstC -= dims.width;
    }
  }

  // RTCLogInfo(@"Processed depth with min=%d, max=%d", min, max);

  status = CVPixelBufferUnlockBaseAddress(depthData.depthDataMap, 0);
  if (status != noErr) {
    RTCLogError(@"CVPixelBufferUnlockBaseAddress failed: %d", status);
  }
  status = CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
  if (status != noErr) {
    RTCLogError(@"CVPixelBufferUnlockBaseAddress failed: %d", status);
  }

  status = CMVideoFormatDescriptionCreateForImageBuffer(NULL, imageBuffer, &desc);
  if (status != noErr) {
    RTCLogError(@"CMVideoFormatDescriptionCreateForImageBuffer failed to set: %d", status);
  }

  CMSampleBufferRef sampleBuffer;
  status = CMSampleBufferCreateReadyWithImageBuffer(
    kCFAllocatorDefault,
    imageBuffer,
    desc,
    &timing,
    &sampleBuffer
  );
  if (status != noErr) {
    RTCLogError(@"CMSampleBufferCreateReadyWithImageBuffer failed to set: %d", status);
  }

  _capturer->CaptureSampleBuffer(sampleBuffer, rotation);

  CFRelease(sampleBuffer);
  CFRelease(imageBuffer);
}

- (void)depthDataOutput:(AVCaptureDepthDataOutput *)output
       didDropDepthData:(AVDepthData *)depthData
              timestamp:(CMTime)timestamp
             connection:(AVCaptureConnection *)connection
                 reason:(AVCaptureOutputDataDroppedReason)reason {
  RTCLogError(@"Dropped depthData buffer.");
}

#pragma mark - AVCaptureSession notifications

- (void)handleCaptureSessionInterruption:(NSNotification *)notification {
  NSString *reasonString = nil;
#if defined(__IPHONE_9_0) && defined(__IPHONE_OS_VERSION_MAX_ALLOWED) && \
    __IPHONE_OS_VERSION_MAX_ALLOWED >= __IPHONE_9_0
  if ([UIDevice isIOS9OrLater]) {
    NSNumber *reason = notification.userInfo[AVCaptureSessionInterruptionReasonKey];
    if (reason) {
      switch (reason.intValue) {
        case AVCaptureSessionInterruptionReasonVideoDeviceNotAvailableInBackground:
          reasonString = @"VideoDeviceNotAvailableInBackground";
          break;
        case AVCaptureSessionInterruptionReasonAudioDeviceInUseByAnotherClient:
          reasonString = @"AudioDeviceInUseByAnotherClient";
          break;
        case AVCaptureSessionInterruptionReasonVideoDeviceInUseByAnotherClient:
          reasonString = @"VideoDeviceInUseByAnotherClient";
          break;
        case AVCaptureSessionInterruptionReasonVideoDeviceNotAvailableWithMultipleForegroundApps:
          reasonString = @"VideoDeviceNotAvailableWithMultipleForegroundApps";
          break;
      }
    }
  }
#endif
  RTCLog(@"Capture session interrupted: %@", reasonString);
  // TODO(tkchin): Handle this case.
}

- (void)handleCaptureSessionInterruptionEnded:(NSNotification *)notification {
  RTCLog(@"Capture session interruption ended.");
  // TODO(tkchin): Handle this case.
}

- (void)handleCaptureSessionRuntimeError:(NSNotification *)notification {
  NSError *error = [notification.userInfo objectForKey:AVCaptureSessionErrorKey];
  RTCLogError(@"Capture session runtime error: %@", error);

  [RTCDispatcher dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                               block:^{
#if TARGET_OS_IPHONE
                                   if (error.code == AVErrorMediaServicesWereReset) {
                                     [self handleNonFatalError];
                                   } else {
                                     [self handleFatalError];
                                   }
#else
                                   [self handleFatalError];
#endif
                               }];
}

- (void)handleCaptureSessionDidStartRunning:(NSNotification *)notification {
  RTCLog(@"Capture session started.");

  self.isRunning = YES;
  [RTCDispatcher dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                               block:^{
                                   // If we successfully restarted after an unknown error,
                                   // allow future retries on fatal errors.
                                   _hasRetriedOnFatalError = NO;
                               }];
}

- (void)handleCaptureSessionDidStopRunning:(NSNotification *)notification {
  RTCLog(@"Capture session stopped.");
  self.isRunning = NO;
}

- (void)handleFatalError {
  [RTCDispatcher
      dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                    block:^{
                        if (!_hasRetriedOnFatalError) {
                          RTCLogWarning(@"Attempting to recover from fatal capture error.");
                          [self handleNonFatalError];
                          _hasRetriedOnFatalError = YES;
                        } else {
                          RTCLogError(@"Previous fatal error recovery failed.");
                        }
                    }];
}

- (void)handleNonFatalError {
  [RTCDispatcher dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                               block:^{
                                   if (self.hasStarted) {
                                     RTCLog(@"Restarting capture session after error.");
                                     [self.captureSession startRunning];
                                   }
                               }];
}

#if TARGET_OS_IPHONE

#pragma mark - UIApplication notifications

- (void)handleApplicationDidBecomeActive:(NSNotification *)notification {
  [RTCDispatcher dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                               block:^{
                                   if (self.hasStarted && !self.captureSession.isRunning) {
                                     RTCLog(@"Restarting capture session on active.");
                                     [self.captureSession startRunning];
                                   }
                               }];
}

#endif  // TARGET_OS_IPHONE

#pragma mark - Private

- (BOOL)setupCaptureSession {
  AVCaptureSession *captureSession = [[AVCaptureSession alloc] init];
#if defined(WEBRTC_IOS)
  captureSession.usesApplicationAudioSession = NO;
#endif

/*
  // XXX: for now, just capture depth.

  // Add the output.
  AVCaptureVideoDataOutput *videoDataOutput = [self getVideoDataOutput];
  if (![captureSession canAddOutput:videoDataOutput]) {
    RTCLogError(@"Video data output unsupported.");
    return NO;
  }
  [captureSession addOutput:videoDataOutput];
*/

  // Add the depth output.
  AVCaptureDepthDataOutput *depthDataOutput = [self getDepthDataOutput];
  if (![captureSession canAddOutput:depthDataOutput]) {
    RTCLogError(@"Depth data output unsupported.");
    return NO;
  }
  [captureSession addOutput:depthDataOutput];

  // Get the front and back cameras. If there isn't a front camera
  // give up.
  AVCaptureDeviceInput *frontCameraInput = [self frontCameraInput];
  AVCaptureDeviceInput *backCameraInput = [self backCameraInput];
  if (!frontCameraInput) {
    RTCLogError(@"No front camera for capture session.");
    return NO;
  }

  // Add the inputs.
  if (![captureSession canAddInput:frontCameraInput] ||
      (backCameraInput && ![captureSession canAddInput:backCameraInput])) {
    RTCLogError(@"Session does not support capture inputs.");
    return NO;
  }
  AVCaptureDeviceInput *input = self.useBackCamera ? backCameraInput : frontCameraInput;
  [captureSession addInput:input];

  _captureSession = captureSession;
  return YES;
}

- (AVCaptureVideoDataOutput *)getVideoDataOutput {
  if (!_videoDataOutput) {
    // Make the capturer output NV12. Ideally we want I420 but that's not
    // currently supported on iPhone / iPad.
    AVCaptureVideoDataOutput *videoDataOutput = [[AVCaptureVideoDataOutput alloc] init];
    videoDataOutput.videoSettings = @{
      (NSString *)
      // TODO(denicija): Remove this color conversion and use the original capture format directly.
      kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange)
    };
    videoDataOutput.alwaysDiscardsLateVideoFrames = NO;
    [videoDataOutput setSampleBufferDelegate:self queue:self.frameQueue];
    _videoDataOutput = videoDataOutput;
  }
  return _videoDataOutput;
}

- (AVCaptureDepthDataOutput *)getDepthDataOutput {
  if (!_depthDataOutput) {
    AVCaptureDepthDataOutput *depthDataOutput = [[AVCaptureDepthDataOutput alloc] init];
    depthDataOutput.alwaysDiscardsLateDepthData = NO;
    depthDataOutput.filteringEnabled = YES;
    [depthDataOutput setDelegate:self callbackQueue:self.frameQueue];
    _depthDataOutput = depthDataOutput;
  }
  return _depthDataOutput;
}

- (AVCaptureDevice *)videoCaptureDeviceForPosition:(AVCaptureDevicePosition)position {
 AVCaptureDevice *device;
  device = [AVCaptureDevice defaultDeviceWithDeviceType: AVCaptureDeviceTypeBuiltInTrueDepthCamera
                                              mediaType: AVMediaTypeDepthData
                                               position: position];
  if (device != nil) {
    return device;
  }
  device = [AVCaptureDevice defaultDeviceWithDeviceType: AVCaptureDeviceTypeBuiltInDuoCamera
                                              mediaType: AVMediaTypeDepthData
                                               position: position];
  if (device != nil) {
    return device;
  }

/*  
  for (AVCaptureDevice *captureDevice in [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo]) {
    if (captureDevice.position == position) {
      return captureDevice;
    }
  }
*/  
  return nil;
}

- (AVCaptureDeviceInput *)frontCameraInput {
  if (!_frontCameraInput) {
#if TARGET_OS_IPHONE
    AVCaptureDevice *frontCameraDevice =
        [self videoCaptureDeviceForPosition:AVCaptureDevicePositionFront];
#else
    AVCaptureDevice *frontCameraDevice =
        [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
#endif
    if (!frontCameraDevice) {
      RTCLogWarning(@"Failed to find front capture device.");
      return nil;
    }
    NSError *error = nil;
    AVCaptureDeviceInput *frontCameraInput =
        [AVCaptureDeviceInput deviceInputWithDevice:frontCameraDevice error:&error];
    if (!frontCameraInput) {
      RTCLogError(@"Failed to create front camera input: %@", error.localizedDescription);
      return nil;
    }
    _frontCameraInput = frontCameraInput;
  }
  return _frontCameraInput;
}

- (AVCaptureDeviceInput *)backCameraInput {
  if (!_backCameraInput) {
    AVCaptureDevice *backCameraDevice =
        [self videoCaptureDeviceForPosition:AVCaptureDevicePositionBack];
    if (!backCameraDevice) {
      RTCLogWarning(@"Failed to find front capture device.");
      return nil;
    }
    NSError *error = nil;
    AVCaptureDeviceInput *backCameraInput =
        [AVCaptureDeviceInput deviceInputWithDevice:backCameraDevice error:&error];
    if (!backCameraInput) {
      RTCLogError(@"Failed to create front camera input: %@", error.localizedDescription);
      return nil;
    }
    _backCameraInput = backCameraInput;
  }
  return _backCameraInput;
}

// Called from capture session queue.
- (void)updateOrientation {
#if TARGET_OS_IPHONE
  _orientation = [UIDevice currentDevice].orientation;
#endif
}

// Update the current session input to match what's stored in _useBackCamera.
- (void)updateSessionInputForUseBackCamera:(BOOL)useBackCamera {
  [RTCDispatcher
      dispatchAsyncOnType:RTCDispatcherTypeCaptureSession
                    block:^{
                      [_captureSession beginConfiguration];
                      AVCaptureDeviceInput *oldInput = _backCameraInput;
                      AVCaptureDeviceInput *newInput = _frontCameraInput;
                      if (useBackCamera) {
                        oldInput = _frontCameraInput;
                        newInput = _backCameraInput;
                      }
                      if (oldInput) {
                        // Ok to remove this even if it's not attached. Will be no-op.
                        [_captureSession removeInput:oldInput];
                      }
                      if (newInput) {
                        [_captureSession addInput:newInput];
                      }
                      [self updateOrientation];
                      AVCaptureDevice *newDevice = newInput.device;
                      const cricket::VideoFormat *format = _capturer->GetCaptureFormat();
                      webrtc::SetFormatForCaptureDevice(newDevice, _captureSession, *format);
                      [_captureSession commitConfiguration];
                    }];
}

@end
