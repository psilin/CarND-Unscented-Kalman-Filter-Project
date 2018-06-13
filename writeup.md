## Writeup

---

**Unscented Kalman Filter Project Project**

[//]: # (Image References)
[image1]: ./pics/All_set1.png
[image2]: ./pics/All_set2.png
[image3]: ./pics/Lidar_only_set1.png
[image4]: ./pics/Radar_only_set1.png
[image5]: ./pics/lidar_nis.png
[image6]: ./pics/radar_nis.png


## [Rubric](https://review.udacity.com/#!/rubrics/783/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 0. General discussion

Code is provided with writeup.md or available [here](https://github.com/psilin/CarND-Unscented-Kalman-Filter-Project/). I have not made any changes to standard `cmake`-based building process
as I ended up only filling some methods.


#### 1. Algorithm

Firstly, I implemented `CalculateRMSE()` helper method of `Tools` class (`src/tools.cpp`). I just reused the same method from Extended Kalman Filter project.

Then I filled methods of `ProcessMeasurement()`, `Prediction()`, `UpdateLidar()` and `UpdateRadar()` of `UKF` class (`src/ukf.cpp`). In `ProcessMeasurement()` (`lines 76-133`) I did filter 
initialization on a first epoch and called prediction and measurement update methods (depending on what type of measurements was received on a given epoch). In case of lidar 
based initialization I only initialized `px` and `py` of a state vector as lidar only gave position measurements. In case of radar I also initialized `phi` and initialized `v` parameter
of state vector with `rhodot` as in my opinion it was better to initialize speed with only radial projection then with zero. I also made sure that UKF would not do a second prediction step
in case to measurement updates (from radar and from lidar) were received on the same time epoch.

In `Prediction()` (`lines 140-239`) I started with creation of sigma points (`lines 147-176`). I used augmentation approach from lesson and used covariance matrix `P_` and process noise
matrix to create augmented sigma points. So I ended up with `1 + 2 * (5 + 2) = 15` points, where `5x5` is the size of covariance matrix and `2x2` is the size of process noise matrix.
Then I used `CTRV` model to predict how every sigma point would change given a `dt` change in time (`lines 177-221`). I just updated state vector using `CTRV` equations and added noise.
Then I used those predicted sigma point to obtain predicted mean vector and covariance matrix (`lines 222-239`). I also implemented angle normalization as it was very important when working
with state elements that were in radians.

Then I moved to `UpdateRadar()` method (`lines 324-424`). That method was based on two major steps. First, I projected predicted sigma points to measurement space (`lines 334-354`). 
Rest of the method was used to calculate difference between those projected sigma points or in other words predicted measurements and actuall measurement from sensor. Those difference
vectors were used to obtain Kalman gain matrix `K` that was used to update state mean and covariance matrix. I did not forget to implement angle normalization there as well. As for
`UpdateLidar()` method (`lines 245-318`) it was almost the same but easier to imlement as it was easier to project predicted sigma points to lidar measurment space than to radar 
measurement space. For both update methods I implemented some code to collect `NIS` statistics for both methods to check if sensor noise matrix were good.

I decided against of refactoring `UKF` class and introducing some private methods as all methods were not more that `100` lines of code and in my opinion it was hard to extract some common
part without introducing some unnecessary complexity. It is important to mention that I heavily used code from lesson in fact I only did some minor tweaks to make it work.

Then I moved to initialization phase. I initialized covariance matrix `P_` with identity matrix as it was suggested in lesson. Then I moved to process noise initialization, more 
specifically, longitudal acceleration and yaw acceleration noise. Both were set to `30` and it was clear that it was to big. I decided to start with setting both parameters equal to
each other and trying to find a value for both parameters that would produce good results. Then made some additional tweaks if necessary. I made following steps 
`30.0 -> 6.0 -> 4.0 -> 3.0 -> 2.0 -> 1.0 -> 0.1`. I realized that the less value I use for noize the better results I get. At `3.0` the result were almost acceptable so I moved to
`2.0` and got better results. Then I moved to `1.0` and got even better results. I tried to further decrease noise parameters to `0.1` but with no success so I ended up using
`1.0` for both noise parameters.

#### 2. Accuracy

So with noise parameters set as was described above I got the following result on first data set:

![alt text][image1]

That result was acceptable interms of RMSE `(0.0647, 0.0837, 0.3353, 0.2194)` and was smaller that desired RMSE of `(0.09, 0.10, 0.40, 0.33)` and more importantly the resulting estimate 
was smooth had no spikes.

I tried to use it on second set and obtained RMSE of `(0.0661, 0.0765, 0.3447, 0.3051)`:

![alt text][image2]

I performend an estimation with only radar available on first data set:

![alt text][image4]

I performend an estimation with only lidar available on first data set as well:

![alt text][image3]

It can be seen that using both sensors results in better performance (as we have more information about system). Lidar-only results looks better then radar-only as lidar produces less
noisy measurements then radar. Though radar helps to improve lidar-only solution. My opinion is the biggest advantage of using radar is the fact that it provides direct radial speed 
measurements while lidar does not measure speed in direct way. That is why it is reaaly helpful to use both sensors.

#### 3. Discussion

I calculated `NIS` statistic (based on `chi2` distribution) for lidar:

![alt text][image5]

and radar:

![alt text][image6]

It can be seen that in both cases percentage of ouliers is less than `5%`. It means that for both lidar and radar measurement noise parameters were chosen correctly which is of
no surprise as those parameters were provided with code.

I compared result of UKF (RMSE `(0.0647, 0.0837, 0.3353, 0.2194)`) with results that I obtained in EKF project (RMSE `(0.0974, 0.0855, 0.4517, 0.4404)`). As expected UKF performs better,
the most improvement we got in estimation of speed. It is of no surprise as we used a more sophisticated motion model (`CTRV`) than in EKF project.

As for what can be improved. This UKF-based solution is certainly more suitable for autonomous car then EKF-based one. But it can have difficulties in case of big accelerations. Other
thing is computatonal stability. We use a square root of augmented covariance matrix to obtain sigma points. We need to make sure that if we somehow resulted with covariance matrix
that do not have a square root matrix we need to modify covariance matrix in a way that allows us to compute a square root matrix (for example, by adding so little noise to covariance
matrix).

Second suggestion is to implement some sort of catchers that can help in finding anomalies in measurements. If we can find and discard such measurements it can really improve robustness
of our solution.
