# EKF Process Diagram

This file provides Mermaid diagrams for the current repository EKF flow. It complements `EKF_Explanation.tex` and `EKF_Explanation.md`.

## Software Architecture

`DynamicsModel.py` and `ExtendedKalmanFilter.py` are intentionally separate.
The dynamics model owns **how the car moves** (`f(x,u)`).  The EKF owns
**how confident we are** (`P`) and applies **sensor corrections**.

```mermaid
%%{init: {'flowchart': {'curve': 'step'}}}%%
flowchart LR
    CFG["config/\nvehicle.yaml\nekf.yaml\nsensors.yaml"]
    DM["DynamicsModel\nf(x,u) — process model\nslip angles, tire forces\nload transfer, handling"]
    EKF["ExtendedKalmanFilter\nstate x, covariance P\npredict + update"]
    GPS["GPS\n(position, velocity)"]
    IMU["IMU BNO055\n(yaw, yaw rate)"]
    RC["RC Receiver\n(steering angle)"]
    OUT["Estimate\nx_k^+, P_k^+"]

    CFG --> DM
    CFG --> EKF
    DM -->|"f_d(x,u)  F_k"| EKF
    GPS --> EKF
    IMU --> EKF
    RC -->|"delta — predict input"| EKF
    EKF --> OUT
```

## High-Level EKF Flow

```mermaid
%%{init: {'flowchart': {'curve': 'step'}}}%%
flowchart LR
    A["Current state x_k-1^+"] --> B["Dynamics model f(x,u)"]
    U["Input u_k"] --> B
    B --> C["Predicted state x_k^-"]
    C --> D[Linearize process]
    D --> E[F_k]
    E --> F["Predicted covariance P_k^-"]

    C --> G["Measurement model h(x)"]
    G --> H["Predicted measurement h(x_k^-)"]
    H --> I["Innovation y_k = z_k - h(x_k^-)"]
    Z["Sensor measurement z_k"] --> I
    I --> J["Innovation covariance S_k"]
    J --> K["Kalman gain K_k"]
    K --> L["Corrected state x_k^+"]
    K --> M["Corrected covariance P_k^+"]
```

## Process Model Versus Measurement Model

```mermaid
%%{init: {'flowchart': {'curve': 'step'}}}%%
flowchart LR
    A["Current state x_k-1^+\nand covariance P_k-1^+"] --> B["DynamicsModel\nf_d(x,u)"]
    U["Input u_k\n[a_x_meas, delta]"] --> B
    B --> C["Predicted state x_k^-\n(forward Euler)"]
    C --> D["Numerical Jacobian\ncentral differences"]
    D --> E["F_k = d f_d / d x"]
    E --> F["P_k^- = F_k P_k-1^+ F_k^T + Q_k"]

    C --> G["Sensor model h(x)\nanalytic or numerical H_k"]
    G --> H["Predicted measurement h(x_k^-)"]
    H --> I["Innovation y_k = z_k - h(x_k^-)"]
    Z["Sensor measurement z_k\n(GPS pos/vel, IMU yaw/rate)"] --> I
    I --> J["S_k = H_k P_k^- H_k^T + R_k"]
    J --> K["K_k = P_k^- H_k^T S_k^-1"]
    K --> L["x_k^+ = x_k^- + K_k y_k"]
    K --> M["P_k^+ = Joseph form"]
```

## Sensor Measurement Models (H matrices)

Each sensor update uses a different `h(x)` function and its Jacobian `H_k`.

```mermaid
%%{init: {'flowchart': {'curve': 'step'}}}%%
flowchart LR
    X["State x_k^-\n[pe, pn, psi, vx, vy, r, bax, bgz]"]

    X --> H1["h_gps_pos\nh = [pe, pn]\nH analytic — identity rows"]
    X --> H2["h_gps_vel\nh = R(psi)[vx,vy]\nH analytic — involves psi,vx,vy"]
    X --> H3["h_imu_yaw\nh = [psi]\nH analytic — identity row"]
    X --> H4["h_imu_yaw_rate\nh = [r + bgz]\nH analytic — identity row"]

    H1 --> R1["R_gps_pos = diag(2.5^2, 2.5^2) m^2"]
    H2 --> R2["R_gps_vel = diag(0.05^2, 0.05^2) m^2/s^2"]
    H3 --> R3["R_yaw = [0.026^2] rad^2"]
    H4 --> R4["R_rate = [0.0014^2] rad^2/s^2"]
```

## What F_k Means

`F_k` is the state-transition Jacobian:

```text
F_k = d f_d / d x
```

It answers this question:

```text
If the current state estimate changes a little,
how does the predicted next state change?
```

That is why it appears in:

```text
P_k^- = F_k P_k-1^+ F_k^T + Q_k
```
