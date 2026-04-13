# EKF Process Diagram

This file provides Mermaid diagrams for the current repository EKF flow. It complements `EKF_Explanation.tex` and `EKF_Explanation.md`.

## High-Level EKF Flow

```mermaid
flowchart TD
    A[Load config bundle] --> B[Build DynamicsModel]
    B --> C[Build ExtendedKalmanFilter]
    C --> D[Wait for first good GPS fix]
    D --> E[Set local ENU origin]
    E --> F[Initialize state x0 and covariance P0]
    F --> G[Enter main loop]

    G --> H[Read dt, steering, IMU accel x, IMU gyro z, optional yaw, GPS]
    H --> I[Build input u_k and convert steering sign]
    I --> J[Predict with process model]
    J --> J1["x_k^- = f_d(x_k-1^+, u_k)"]
    J1 --> J2["F_k = d f_d / d x"]
    J2 --> J3["P_k^- = F_k P_k-1^+ F_k^T + Q_k"]

    J3 --> K{GPS position available?}
    K -- Yes --> L[Update GPS position]
    K -- No --> M{GPS speed and track available?}
    L --> M

    M -- Yes --> N{Speed above threshold?}
    M -- No --> O{Yaw measurement available?}
    N -- Yes --> P[Update GPS velocity]
    N -- No --> O
    P --> O

    O -- Yes --> Q[Update yaw]
    O -- No --> R[Update yaw rate]
    Q --> R

    R --> S["Publish x_k^+ and P_k^+"]
    S --> G
```

## Process Model Versus Measurement Model

```mermaid
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
