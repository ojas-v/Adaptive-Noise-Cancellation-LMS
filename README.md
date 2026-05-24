# Software-Defined Adaptive Noise Cancellation (ANC) 🎧

![MATLAB](https://img.shields.io/badge/MATLAB-Simulation-blue)
![Simulink](https://img.shields.io/badge/Simulink-DSP_Architecture-blue)
![Status](https://img.shields.io/badge/Status-Phases_1_to_5_Complete-success)

This is a complete, hardware-targeted simulation of a dual-microphone feedforward Adaptive Noise Cancellation (ANC) system. 

I didn't just model the theoretical math; I built this architecture to be directly flashed to an ARM Cortex-M4 or TI C55xx DSP. The system uses a custom **Normalized Filtered-X LMS (NLMS-FxLMS)** engine to dynamically crush a 200-tap simulated acoustic room environment while maintaining strict real-time causality and surviving 16-bit fixed-point quantization.

### Key Performance Metrics
* **Noise Reduction:** +8.38 dB average Segmental SNR gain (validated via 500-trial Monte Carlo sweep).
* **System ID (ERLE):** 31.98 dB (The digital twin perfectly mapped the physical speaker-to-mic air gap).
* **Silicon-Ready:** -0.03 dB SegSNR penalty when filter weights were quantized to Q15 fixed-point math.
* **Causality Margin:** +26 samples (The firmware calculates the anti-noise 3.25ms faster than the physical speed of sound).

---

## The Engineering Progression

Most academic ANC simulations use a basic LMS filter and assume the anti-noise reaches the user's ear instantly. In the real world, that assumption causes the algorithm to violently diverge. I built this system in phases to explicitly solve the physics of the physical air gap.

### Phase 3 & 4: Standard LMS (The Baseline)
I started by building a standard LMS adaptive filter to cancel an AM-modulated voice signal corrupted by a 200-tap exponential room impulse response (RT60 = 150ms). 
* **The Reality Check:** While it achieved a +3.27 dB SNR gain, standard LMS is blind to the physical propagation delay between the ANC speaker and the error microphone. It hits a mathematical ceiling.

### Phase 5: Filtered-X LMS (The Hardware Reality)
To make this deployable, I separated the physical acoustic plant from the embedded DSP firmware.
1. **Offline System ID:** The system first injects broadband white noise to model the secondary path (speaker-to-error-mic), identifying a 16-tap FIR filter (`S_hat`) that perfectly mirrors the physical hardware.
2. **The "Brain Transplant":** I ripped out the standard LMS block and wrote a custom C-style **Normalized LMS (NLMS)** engine. By filtering the reference noise through `S_hat` *before* updating the weights, the algorithm aligns the digital memory buffer with the physical acoustic phase.
3. **Adaptive Stability:** Standard LMS exploded (Output: NaN) when fighting the 200-tap room due to varying noise powers. The NLMS engine scales the step-size ($\mu$) dynamically based on the input power buffer, locking in mathematical stability.

---

## Proof of Concept: Validation 

<img width="1074" height="855" alt="image" src="https://github.com/user-attachments/assets/8114379e-40ed-4756-aafc-972e2029fd28" />
> **Figure 1: Welch Power Spectral Density.** The blue noise floor is crushed across the 0-2kHz band while preserving the harmonic peaks of the target voice.


<img width="1077" height="857" alt="image" src="https://github.com/user-attachments/assets/1b107305-37fa-49bc-92d0-4bf1b6f5a68e" />
> **Figure 2: Fixed-Point DSP Degradation.** Proof that the calculated weights survive being crushed down to 16-bit Q15 memory registers without the acoustic output falling apart.

---

## Implementation Challenges & Troubleshooting
Building the math was easy; locking it to a simulation clock was brutal. Here is how I solved the hardware-level integration issues:

* **Breaking the Algebraic Loop:** Simulink initially crashed because the weight-update equation depended on the error, and the error depended on the weights. I explicitly inserted a unit delay ($z^{-1}$) block on the weight-update line to mimic the one-clock-cycle memory delay of a physical DSP chip, breaking the loop without desyncing the audio.
* **C-Code Compiler Strictness:** Moving the NLMS math from a MATLAB script into a Simulink C-coder block triggered matrix-dimension crashes. I had to explicitly format the code using element-wise operations (`.*`) to force the DSP to multiply the scalar error against every individual tap in the 64-sample buffer.
* **The Phase Mismatch "Yellow Screen of Death":** Early in Phase 5, I accidentally placed a delay block on the physical audio line rather than the firmware memory line. The algorithm guessed wrong, overcorrected, and amplified the noise to infinity. This proved exactly why the physical causality margin (+26 samples) is the most critical metric in ANC design.

---

## How to Run It
1. Clone this repository and open MATLAB.
2. Run `ANC_Simulation.m` to generate the 200-tap room physics and validate the Phase 4 baseline.
3. Run `FxLMS_Simulation.m` to identify the secondary path, calculate the NLMS stability bounds, and load the workspace arrays.
4. Open `FxLMS_Model.slx` and hit **Run**. Open the Scope to watch the NLMS filter crush the noise floor in real-time.

*Note: Requires MATLAB DSP System Toolbox. Fixed-point analysis requires the Fixed-Point Designer Toolbox.*
