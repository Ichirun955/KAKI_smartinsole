# KAKI Smart Insole Platform
A modular, 3D-printed smart insole platform for lower-body sensing, gait analysis, and health monitoring.

## 🚨 The Problem & Solution
**The Problem:** Despite over 18,000 research papers published on smart insoles in the last two decades, there is no unified standard—sensor placement, quantity, and measurement methods vary widely across studies. Furthermore, existing commercial insoles permanently integrate delicate sensors with processing electronics, leading to high replacement costs, e-waste, and poor adaptability.

**The Solution:** KAKI addresses this by completely separating a customizable, passive 3D-printed sensing footbed from a reusable, shoe-mounted electronic "smart pod."

## 🏆 Achievements & Awards
* **22nd Business Planning Contest at KUAS (Dec 2024)**: Grand Prize Winner for the "KAKI smart insole" proposal.
* **Tokyo Global Innovation Challenge (TGIC) 2025**: Innovation and Potential Award.
* **25th Higashi-Mikawa Business Plan Contest (FY2025)**: Special Award (Idea Category) for the "KAKI Insole: Core of Smart Sports Tourism" proposal.
* **iCAN'26 (International Contest of InnovAtioN) Japan (Apr 2026)**: Special Award (Technical Category).
* **IEEE GCCE (Oct 2026)**: Oral presentation and published research.

## 📄 Academic Publication
If you use this hardware or software in your research, please cite our paper:
> Ichiro Nathanael Wiratma, Jonathan Richard Santoso, Alban Mulky Adiguna Turnip, Ian Piumarta, Kayla Narina Tjandra, and Zilu Liang. "KAKI: A Modular Smart Insole Platform for Lower-Body Sensing and Health Monitoring." *2026 IEEE Global Conference on Consumer Electronics (GCCE)*.

*(A `CITATION.cff` file is included in this repository to automatically generate citations).*

## ⚙️ Hardware Components (`/hardware`)
The physical insole separates the passive sensing footbed from the active electronics.
* **Smart Pod:** Seeed Studio XIAO nRF52840 Sense, MPR121 Capacitive Touch Sensor Controller, BNO055 9-DOF IMU, LPS22DF Barometric Sensor.
* <img width="1917" height="1078" alt="image" src="https://github.com/user-attachments/assets/e290356c-ddfd-40a5-bec7-d35f61bd5885" />

* **3D Printed Footbed:** 85A TPU gyroid dielectric (variable infill for sensitivity tuning) and a 90A TPU encapsulation. STEP and STL files are provided.

## 💻 Software & Firmware (`/software` & `/firmware`)
<img width="1915" height="1078" alt="image" src="https://github.com/user-attachments/assets/018b2184-50d1-47a9-9802-13f515a1a38e" />

* **Firmware:** C++ code for the XIAO nRF52840 to poll the I2C sensor array and transmit data via BLE UART at 100 Hz.
* **Software:** Python scripts for host-side processing, including BLE reception, jitter buffering, and gait event detection.


### Setup Instructions
1. Clone this repository: `git clone https://github.com/Ichirun955/KAKI_smartinsole.git`
2. Set up the Conda environment: `conda create --name kaki_env python=3.10`
3. Install dependencies: `pip install -r requirements.txt`

## 📊 10-Meter Walk Test Dataset (`/data`)
This repository includes anonymized dynamic walking data to provide an initial assessment of sensor behavior under different loading conditions. 

* **Participants & Sessions:** The preliminary evaluation involved three participants and five recording sessions, including three sessions from Participant 1 and one session each from Participants 2 and 3 at slow, normal, and jogging speeds. 
* **Setup:** The KAKI smart insole was worn on the left foot and recorded plantar pressure and onboard IMU data.
* **Clinical Protocol:** The datasets follow a standardized clinical 14-meter pathway (2m acceleration, 10m steady-state data collection, 2m deceleration) to isolate true steady-state velocity.
* <img width="1917" height="1078" alt="image" src="https://github.com/user-attachments/assets/7a03535e-8f1d-4a95-a70d-751b6a36c709" />

* **Ethics Statement:** Ethics approval was obtained from the Ethics Review Board of Kyoto University of Advanced Science.
* **Structure:** Each trial folder contains synchronized 100 Hz CSV telemetry from the Insole, a smartphone (Phyphox), and a smartwatch (RawLogger). A `ground_truth.txt` file is included in each folder containing visually verified footstrike times for algorithm validation.
