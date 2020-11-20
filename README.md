# Control 

## Problem 3
![](https://github.com/jinmingteo/CMU_Robotics_HW1/blob/master/outputs/HW1_Problem3.gif)

Observed that red sphere (Impedance Control) experience more steep spikes than the blue sphere (Force Control). It seems that the Integral Gain is more stable than Derivative Gain when we are using PID controller. For impedance control, there is also a manual tweak of the PID_D and PID_P such that it stabilizes at the Fdes (10N).


## Problem 4
![](https://github.com/jinmingteo/CMU_Robotics_HW1/blob/master/outputs/Hw1_Problem4.gif)

Observed that red sphere (Impedance Control) is less stable than blue sphere (Force Control). Since Integral Gain is cumulative, the green sphere is able to handle the slope better than red sphere. As aforementioned, the Fdes in red sphere is achieved by tweaking the PID_D and PID_P. As the situation has changed, red sphere becomes unstable and does not achieve Fdes (10N). I speculate that it still achieves 10N sometimes because at that position, its setting is similar to the initial setting (where the plane was flat).


# Kinematics

## Results for Forward Kinematics

```
computed FK ee position [5.170250e-01 1.000000e-04 4.193268e-01]
computed FK ee rotation [[1. 0. 0.]
 [0. 1. 0.]
 [0. 0. 1.]]
computed FK ee position [ 0.34547502 -0.25488326  0.3462286 ]
computed FK ee rotation [[ 0.66446302  0.4166168  -0.62041867]
 [-0.66446302 -0.0505914  -0.74560673]
 [-0.34202014  0.90767337  0.24321035]]
computed FK ee position [0.55687573 0.26545164 0.18333242]
computed FK ee rotation [[ 0.66341395 -0.5         0.5566704 ]
 [ 0.38302222  0.8660254   0.3213938 ]
 [-0.64278761  0.          0.76604444]]
 ```

## Results and Observation for Inverse Kinematics

In general, Jacobian Transpose Approach is less computationally expensive than Jacobian Pseudo-Inverse Approach

Tweaked with some hyperparameters as shown below:

## Jacobian Transpose Approach

**Hyperparameters setting: loops 1000, limit_angle 0.1, alpha 0.1**
```
Approach:Jacobian Transpose Approach
Total Abs Error: 0.13807912461430172, Abs Median Error: 0.006901961033653881
error [0.009360746517437974, 0.0019343400404541633, 0.002938495064513233, 0.09746224511799123, 0.0044431755498697875, 0.021940122324035347]
compute IK angles [-0.09606874701862678, -0.5850859231035256, 0.7973822245426152, 0.23154683393028871, 3.1639644052649887]
```

**Hyperparameters setting: loops 10000, limit_angle 0.1, alpha 0.1**
``
Approach:Jacobian Transpose Approach
Total Abs Error: 0.11829061786731779, Abs Median Error: 0.00033658538183557415
error [0.00018123491382704415, 0.00041740575682060925, 0.00025576500685053905, 0.09811752141887632, 6.726982554165487e-06, 0.0193119637883891]
compute IK angles [-0.001215176695611201, 0.09539020768769986, 0.3680871389499153, -0.07479485047614949, 3.146984267474857]```

**Hyperparameters setting: loops 10000, limit_angle 0.01, alpha 0.1**
```
Approach:Jacobian Transpose Approach
Total Abs Error: 0.011808715167622032, Abs Median Error: 2.288298508855607e-05
error [3.68987240462193e-06, 4.207609777249021e-05, 3.488439258070297e-06, 0.009807866268664601, 7.59109551959597e-07, 0.0019508353799702895]
compute IK angles [-0.000144368228489686, 0.09498843523676924, 0.3665830615815849, -0.06888693130378308, 3.1415487674720466]
```

**Hyperparameters setting: loops 10000, limit_angle 0.01, alpha 0.5**
```
Approach:Jacobian Transpose Approach
Total Abs Error: 0.011975555212041161, Abs Median Error: 0.00011383209239018558
error [9.166202157406822e-06, 0.00021464503406986852, 1.3019150710502636e-05, 0.009813292852347865, 2.0809742103706645e-06, 0.0019233509985451486]
compute IK angles [-0.0007625494809026521, 0.095230848162486, 0.3710434376227905, -0.07919241704598484, 3.143098368697456]
```

**Hyperparameters setting: loops 10000, limit_angle 0.01, alpha 0.01**
```
Approach:Jacobian Transpose Approach
Total Abs Error: 0.027708430825211905, Abs Median Error: 0.003987948855248717
error [0.00796910244901244, 0.0011276927632017499, 0.005934788554697321, 0.009749257731800118, 0.0020411091558001125, 0.0008864801707001654]
compute IK angles [-0.07757583708534484, -0.6398282165472672, 0.7420880792654014, 0.1718788153413152, 1.3680681840221756]
```

**Hyperparameters setting: loops 10000, limit_angle 0.01, alpha 0.05**
```
Approach:Jacobian Transpose Approach
Total Abs Error: 0.023987299252384464, Abs Median Error: 0.002119609718634405
error [0.009789175108252854, 0.0001914425324580923, 0.002033568404733109, 0.009753712856083056, 1.3749318321655154e-05, 0.002205651032535701]
compute IK angles [-0.0030297283880882844, -0.14218482974022126, 0.5638644778450475, 0.023101730286085805, 3.142540311383821]
```

#### Optimal Hyperparameters: **Hyperparameters setting: loops 10000, limit_angle 0.01, alpha 0.1**
****

## Jacobian Pseudo-Inverse Approach

**Hyperparameters setting: loops 1000, limit_angle 0.1**
```
Approach:Jacobian Pseudo-Inverse Approach
Total Abs Error: 0.10349354543008922, Abs Median Error: 0.00022576427101726355
error [4.051770797763865e-07, 0.0002636934597686344, 2.2119405043707907e-06, 0.09995215679923278, 0.00018783508226589269, 0.0030872429712377712]
compute IK angles [-0.0012140738066626134, 0.18956988191271737, 0.5565455378695232, -0.6843589544290509, 3.118779000535312]
```

**Hyperparameters setting: loops 10000, limit_angle 0.1**
```
Approach:Jacobian Pseudo-Inverse Approach
Total Abs Error: 0.10130347211396677, Abs Median Error: 0.0002532084623287819
error [4.84794147714851e-07, 0.00045474255439527237, 2.268387173320896e-06, 0.09999680660320513, 5.16743702622915e-05, 0.0007974954047830413]
compute IK angles [0.0006022163851047782, 0.23428130332270194, 0.5756038908390149, -0.8258413188434011, 3.118858047119461]
```

**Hyperparameters setting: loops 10000, limit_angle 0.01**
```
Approach:Jacobian Pseudo-Inverse Approach
Total Abs Error: 0.011311276644109098, Abs Median Error: 9.988481034243173e-05
error [2.1295729213388626e-07, 0.00019362500838098019, 3.740546665742528e-07, 0.009930028012584816, 6.1446123038832706e-06, 0.0011808919988807098]
compute IK angles [0.0003110640524864024, 0.11888369059774628, 0.47686479072870985, -0.35901460629780196, 3.1479062755948597]
```

**Hyperparameters setting: loops 10000, limit_angle 0.5**
```
Approach:Jacobian Pseudo-Inverse Approach
Total Abs Error: 0.5297493292026579, Abs Median Error: 0.0050594743074165675
error [4.559903114159658e-05, 0.00495807591546746, 0.00034568407266641343, 0.49958698946571667, 0.005160872699365674, 0.019652108018300047]
compute IK angles [-0.011758252328045794, 0.2793029084426769, 0.5838421421130388, -0.9420462766634531, 3.33619278477581]
```

### Optimal Hyperparameters: **Hyperparameters setting: loops 10000, limit_angle 0.01**

****

# Conclusion
It seems that both approaches have the same optimal hyperparameters for this set of problems. Jacobian Pseudo-Inverse approach more accurate than Jacobian Transpose Approach but it is more computationally expensive.
