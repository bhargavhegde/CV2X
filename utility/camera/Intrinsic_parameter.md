## Intrinsic Parameter Estimation

Remark: this file is generated through the interaction with ChatGPT.

One can estimate the **camera intrinsic parameters** from camera + lens specifications as an references to the calibration values.

Here's how the main parameters are derived:

---

### 🔧 Intrinsic Matrix $\mathbf{K}$

$$
\mathbf{K} = 
\begin{bmatrix}
f_x & \gamma & c_x \\\\
0   & f_y     & c_y \\\\
0   & 0       & 1
\end{bmatrix}
$$

Where:

* $f_x = \frac{f \cdot p_x}{s_x}$,  $f_y = \frac{f \cdot p_y}{s_y}$
* $f$: focal length in mm (from lens spec)
* $s_x, s_y$: sensor size in mm (horizontal and vertical)
* $p_x, p_y$: resolution in pixels (width and height)
* $c_x, c_y$: usually center of image (e.g., $W/2, H/2$)
* $\gamma$: skew (often 0 unless sensor is skewed)

---

We can estimate the **intrinsic parameters** for our camera.

---

### ✅ Example: 🔍 Mako G-319C Specs (from Allied Vision)

* **Resolution**: 2064 × 1544 px
* **Sensor Size**: 1/1.8″ format
  → **Approx. 7.13 mm × 5.35 mm** (W × H)
* **Focal Length**: **12 mm lens**

---

### 🧮 Estimate Intrinsics

We convert focal length in mm to pixels:

$$
f_x = \frac{\text{focal length (mm)} \times \text{resolution}_x}{\text{sensor width (mm)}}
$$

$$
f_y = \frac{\text{focal length (mm)} \times \text{resolution}_y}{\text{sensor height (mm)}}
$$

Substitute:

* $f = 12.0$ mm
* $\text{sensor}_x = 7.13$ mm
* $\text{sensor}_y = 5.35$ mm
* $\text{res}_x = 2064$
* $\text{res}_y = 1544$

```python
sensor_width_mm = 7.13
sensor_height_mm = 5.35
res_x = 2064
res_y = 1544
focal_length_mm = 12.0

fx = res_x * (focal_length_mm / sensor_width_mm)  # ≈ 3474.6
fy = res_y * (focal_length_mm / sensor_height_mm) # ≈ 3464.6
cx = res_x / 2                                     # 1032
cy = res_y / 2                                     # 772
```

---

### 📌 Resulting Intrinsic Matrix $K$

$$
K = 
\begin{bmatrix}
3475 & 0 & 1032 \\\\
0 & 3465 & 772 \\\\
0 & 0 & 1
\end{bmatrix}
$$

---
### ✅ Example: 🔍 Mako G-507C Specs (from Allied Vision)

* **Resolution**: 2464 x 2056 px
* **Sensor Size**: 2/3″ format
  → **Approx. 7.448 mm x 5.592 mm** (W × H)
* **Focal Length**: **12 mm lens**

Then:

```python
sensor_width_mm = 7.448
sensor_height_mm = 5.592
res_x = 2464
res_y = 2056
focal_length_mm = 12

fx = res_x * (focal_length_mm / sensor_width_mm)  # ≈ 3970
fy = res_y * (focal_length_mm / sensor_height_mm) # ≈ 4407
cx = res_x / 2                                     # 1232
cy = res_y / 2                                     # 1028
```

---

### 🧠 Summary

| Spec         | Used to estimate...              |
| ------------ | -------------------------------- |
| Focal length | fx, fy via magnification         |
| Sensor size  | scale conversion (mm to pixel)   |
| Resolution   | determines cx, cy (image center) |
| Skew γ       | usually 0 unless known           |

