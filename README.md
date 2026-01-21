# 🎧 Real-Time Sound Source Localization with Visual Tracking Integration

A **real-time audio–visual perception system** that localizes and tracks active sound sources by fusing **microphone-array–based signal processing** with **camera-based visual tracking**. The system improves localization accuracy, robustness, and stability in **dynamic and noisy environments**, and is designed as a **research-oriented foundation** for advanced multi-source localization.

---

## 📌 Project Overview

Sound source localization using microphones alone is highly sensitive to noise, reverberation, and ambiguity—especially in real-world environments. This project addresses these challenges by integrating **Digital Signal Processing (DSP)–based audio localization** with **visual tracking**, enabling more reliable spatial estimation and continuous tracking of sound-emitting objects.

The system combines:
- **Time Difference of Arrival (TDoA)**–based localization from a microphone array  
- **Angle of Arrival (AoA)** and geometry-based spatial inference  
- **Visual cues** to validate, refine, and stabilize audio-based estimates  

This audio–visual fusion framework is suitable for **assistive technologies, robotics, surveillance, and research applications**.

---

## ⚙️ System Functionality & Working

The system operates in real time using a synchronized **multi-microphone array** and a camera module. Audio signals captured by spatially separated microphones are processed frame-by-frame to estimate **inter-microphone time delays**. These delays are converted into **directional and spatial estimates** of the sound source.

To improve robustness, the audio pipeline incorporates **FIR-based filtering and spectral conditioning**, ensuring reliable delay estimation under noise. Simultaneously, a vision pipeline tracks objects within the scene. The **fusion layer associates audio estimates with visual targets**, enabling stable tracking even when one modality degrades.

The result is a system capable of **continuous, real-time sound source localization and tracking**, suitable for extension to **multi-source and probabilistic frameworks**.

---

## 🎯 Objectives

- Localize sound sources in real time using microphone arrays  
- Implement robust DSP algorithms for delay estimation  
- Estimate sound source direction and spatial position  
- Integrate visual tracking to enhance localization stability  
- Develop a scalable architecture suitable for research extensions  

---

## 🧠 System Architecture

