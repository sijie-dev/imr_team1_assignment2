## Q1 DataLogger filename defined?

- No. Line 351 passes only a folder, DataLogger(log_dir="logs"). DataLogger names the file itself from the start time, The name is stored in self.datalog.filename, and stopcommand() prints it at line 386 ("Data saved in ...").


## Q2 Aruco already logged in correct pose format from Line 581, this is where measurement is updated based on Aruco measurement/observation right?

- 586: convert the raw ArUco data to PoseStamped (pose_parse). Nothing is logged here.
- 590–594: save it into measured_pose_*. This only stores the measurement.
- 597: log it under /aruco.

> The **EKF measurement update is not here**. It happens later, at lines 686–696. The only thing near line 597 that changes the estimate is the one-time initialisation at line 602, which copies the first ArUco pose straight into the state.

## Q3 If we got new Aruco measurement ?

- Yes. Line 686 is if self.new_camera_measurement and self.use_measurement_update:. The update runs only when both are true: a new ArUco reading arrived in this loop (the flag is set at line 583), and updates are switched on (line 245). Line 699 resets the flag, so each reading is used only once. Loops without a camera reading do predict only.


## Q4 Here the Covariance used for for process & measurement noise in Line 681 are initialised?

- Yes. Four matrices are defined here, and each is used in a different place:
  - `init_covariance` is the initial Σ₀, used only once during initialization on line 612; `Q` is the process noise, used in the `predict` operation on line 683; `R_ne` and `R_g` are the position and heading noises of ArUco, respectively, used in the `update` operations on lines 691 and 696. Line 686 itself is merely an `if` statement; the actual lines that pass `Q` and `R` into the function are lines 683, 691, and 696.


## Q5 both return predicted & updated Sigma as self.covariance. So is it logged in Line 717 and overwritten once it is updated?

- Yes. Within one loop, the same variable is overwritten in sequence:

``` ASCII
683  predict           → self.covariance = Σ̄       (predicted)
691  update N,E        → self.covariance = Σ'      (only if new ArUco)
696  update yaw        → self.covariance = Σ       (only if new ArUco)
712  self.cov = self.covariance   (for the GUI ellipse)
723  log "/est_cov"    ← logged ONCE per loop, after all of the above
```

> So each /est_cov entry holds the final Σ of that loop. On a loop with a camera reading, that is the posterior after both updates. On a loop without one, it is the predicted Σ̄. On camera loops, the intermediate Σ̄ is not logged.


## Q6 Aren't the data we intended to log (Aruco, LiDar, Sigma) logged already

- Yes. Every loop and every callback already writes to the same JSON file:
    1. /true_wheel_speeds	402
    2. /lidar	450
    3. /aruco	597
    4. /est_pose	719
    5. /est_cov	723
    6. /wheel_speeds_cmd	829



![alt text](image-1.png)

