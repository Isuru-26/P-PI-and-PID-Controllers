# ⚙️ DC Motor Position Control System  
🎯 MATLAB-Based P, PI & PID Controller Analysis with Disturbance Rejection  

---

## 🎯 Project Overview

The **DC Motor Position Control System** is a MATLAB-based control engineering project that focuses on accurately controlling the angular position of a DC motor using **Proportional (P), Proportional–Integral (PI), and Proportional–Integral–Derivative (PID)** controllers.

The objective of the system is to rotate the motor shaft from **0° to 90°** and maintain this position with **high accuracy**, even in the presence of external disturbances. The project includes **mathematical modeling, controller design, auto-tuning, manual tuning, and performance comparison** to identify the most effective control strategy.

---

## ✨ Key Features & Benefits

### 🎯 Accurate Position Control  
Ensures precise motor positioning with minimal steady-state error.

### ⚙️ Multiple Controller Designs  
Implements and compares **P, PI, and PID** controllers under identical conditions.

### 🔄 Auto & Manual PID Tuning  
Uses MATLAB auto-tuning (`pidtune`) followed by **manual gain optimization** for improved response.

### ⚡ Fast Dynamic Response  
PID controller achieves **fast rise time, low overshoot, and short settling time**.

### 🛑 Disturbance Rejection Testing  
Evaluates controller robustness by applying an external disturbance during operation.

### 📊 Visual Performance Analysis  
Clear plots and performance metrics enable easy comparison between controllers.

---

## ⚙️ How It Works

### 1️⃣ DC Motor Modeling
- Electrical and mechanical equations are derived using:
  - Armature resistance and inductance
  - Motor inertia and damping
  - Torque and back-EMF constants
- The system is modeled as a **transfer function** and **state-space model**

### 2️⃣ Controller Design
- P, PI, and PID controllers are designed using MATLAB
- Initial gains are obtained through **auto-tuning**
- PID gains are manually refined to reduce settling time and overshoot

### 3️⃣ Step Response Simulation
- A **90° reference input** is applied
- Performance metrics evaluated:
  - Rise time
  - Overshoot
  - Settling time
  - Steady-state error

### 4️⃣ Disturbance Test
- A step disturbance is introduced at **t = 2 seconds**
- System response and recovery are analyzed for each controller

---

## 🛠️ Technology Stack

| Category | Technologies |
|--------|-------------|
| Simulation Tool | MATLAB |
| Control Design | P, PI, PID Controllers |
| Modeling | Transfer Function, State-Space |
| Tuning Method | Auto-tuning & Manual Tuning |
| Analysis | Step Response, Disturbance Rejection |
| Visualization | MATLAB Plots |

---

## 🚀 Getting Started

### 🔑 Prerequisites
- MATLAB (with Control System Toolbox)
- Basic understanding of control systems and DC motor modeling

---

## ▶️ Run the Simulation

1. Open MATLAB  
2. Load the provided `.m` file  
3. Run the script  
4. Observe:
   - Step responses
   - Disturbance response plots
   - Controller performance comparison

---

## 📊 Performance Summary

| Controller | Rise Time | Overshoot | Settling Time | Steady-State Error |
|-----------|-----------|-----------|---------------|-------------------|
| P | Moderate | High | Long | Present |
| PI | Slow | High | Moderate | Eliminated |
| PID | Fast | Low | Short | Zero |

---

## 💡 Applications

- 🎛️ **DC Motor Position Control**
- 🤖 **Robotics Actuator Control**
- 🏭 **Industrial Automation Systems**
- 🚗 **Servo & Motion Control Applications**
- 🎓 **Control Systems Education & Research**

---

## 🧪 System Behavior Summary

| Condition | P Controller | PI Controller | PID Controller |
|---------|-------------|---------------|----------------|
| Step Tracking | Moderate | Accurate | Excellent |
| Disturbance Response | Poor | Good | Excellent |
| Stability | Moderate | Stable | Highly Stable |

---

## 🔧 Configuration

To modify motor parameters or controller gains, edit the following section in the MATLAB code:

```matlab
J = 0.01;   % Moment of inertia
b = 0.1;    % Damping coefficient
K = 0.01;   % Motor constant
R = 1;      % Resistance
L = 0.5;    % Inductance
