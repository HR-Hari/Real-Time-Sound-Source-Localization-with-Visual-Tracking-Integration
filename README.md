# Real-Time Sound Source Localization with Visual Tracking Integration

A **real-time audio–visual perception system** that localizes and tracks active sound sources by fusing **microphone-array–based signal processing** with **camera-based directional visual tracking**. The system estimates the **2D spatial location (x, y)** of sound sources using audio signals, while visual information is used **only to identify and track the direction of the sound-emitting object**, improving robustness and temporal stability in dynamic environments.

## Project Overview

Sound source localization using microphones alone is highly sensitive to noise, reverberation, and spatial ambiguity. This project addresses these limitations by combining **Digital Signal Processing (DSP)–based audio localization** with **vision-based directional tracking**.

The system performs **metric localization in the (x, y) plane using audio signals**, derived from microphone-array geometry and Time Difference of Arrival (TDoA) measurements. A camera-based vision module operates in parallel to **identify and track the direction of the sound source in the visual field**, enabling validation, refinement, and stabilization of the audio-based estimates.

The system combines:
- **Time Difference of Arrival (TDoA)**–based audio localization  
- **Angle of Arrival (AoA)** and geometry-based **(x, y) position estimation**  
- **Vision-based directional tracking** to associate sound sources with visual targets  

This audio–visual framework is suitable for **assistive technologies, robotics, surveillance, and research applications**.

## System Functionality & Working

The system operates in real time using a synchronized **multi-microphone array** and a camera module. Audio signals captured by spatially separated microphones are processed frame-by-frame to estimate **inter-microphone time delays**. These delays are used to compute the **Angle of Arrival (AoA)** and subsequently infer the **2D spatial location (x, y)** of the sound source using array geometry.

To improve robustness, the audio pipeline incorporates **FIR-based filtering and spectral conditioning**, ensuring reliable delay estimation under noise and reverberation. In parallel, the vision pipeline performs **directional visual tracking**, identifying the angular direction of candidate sound-emitting objects within the camera frame.

The **fusion layer aligns the audio-derived direction and (x, y) position with the vision-based direction**, enabling stable tracking even when one modality is temporarily degraded. Vision is not used for metric distance estimation, but strictly for **directional confirmation and temporal association**.

## Objectives

- Localize sound sources in real time in **2D (x, y) space** using microphone arrays  
- Implement robust DSP algorithms for delay estimation and noise suppression  
- Estimate sound source direction and spatial position using array geometry  
- Use vision-based tracking to identify the **direction** of the sound source  
- Develop a scalable architecture suitable for research extensions  

## System Architecture

    ┌──────────────────────────────┐
    │      Microphone Array        │
    │  (Synchronized Acquisition)  │
    └─────────────▲────────────────┘
                  │
          DSP & Signal Conditioning
                  │
    ┌─────────────┴────────────────┐
    │  TDoA Estimation (GCC-PHAT)  │
    │  AoA & Spatial Computation   │
    └─────────────▲────────────────┘
                  │
          Audio–Visual Fusion
                  │
    ┌─────────────┴────────────────┐
    │     Visual Tracking (Camera) │
    │     Temporal Source Tracking │
    └──────────────────────────────┘
## Implemented DSP Algorithms

- FIR-based noise suppression  
- Spectral conditioning for robust delay estimation  
- Frame-based signal processing  
- Energy-based activity detection  
- Temporal smoothing for stable localization  

## Sound Source Localization Methods

- Time Difference of Arrival (TDoA) estimation using GCC-PHAT  
- Angle of Arrival (AoA) computation using array geometry  
- Geometry-based spatial localization  
- Audio–visual assisted source tracking  

## Hardware & Platforms

| Component | Description |
|---------|------------|
| Microphone Array | MEMS microphones (I2S / TDM) |
| Embedded Platform | ESP32-S3 (real-time audio capture) |
| Camera Module | Visual tracking and scene analysis |
| Host System | PC for processing, visualization, and analysis |

## Software & Technologies

- **Languages**: C / C++, Python, MATLAB  
- **Embedded Development**: ESP-IDF / Arduino  
- **Signal Processing**: FIR filtering, cross-correlation, Amplitiude Normalization and Phase Transfrom (GCC-PHAT) 
- **Computer Vision**: OpenCV, YoloV8  
- **Communication**: Serial
- **Visualization**: Python-based plotting tools  

## Key Features

- Real-time multi-microphone audio acquisition  
- TDoA- and AoA-based sound localization  
- DSP-based noise robustness  
- Visual tracking–assisted localization  
- Temporal smoothing and stable tracking  
- Modular, research-ready architecture  

## Testing & Validation

- TDoA estimation accuracy under varying SNR  
- AoA estimation error analysis  
- Audio-only vs audio–visual localization comparison  
- Temporal stability and tracking consistency  
- Real-time performance evaluation  

## Future Enhancements

- Multi-source sound localization  
- Kalman / Particle filter–based tracking  
- Deep learning–based audio–visual fusion  
- 3D sound source localization  
- Edge AI deployment  
