# Principal Parameters for DKAIL-MPC Reproducibility and Verification

This document summarizes the principal parameters relevant to the released DKAIL-MPC reproducibility and verification materials. The values are divided into manuscript-reported modeling/training settings and final numerical control/learning settings used in the revision experiments.

## 1\. Deep Koopman model and training settings

|Parameter|Value|
|-|-:|
|Learning rate|0.001|
|State-encoder structure|(3, 128, 128, 128, 6)|
|Control-encoder structure|(5, 128, 128, 128, 10)|
|Activation function|ReLU|
|Batch size|128|
|Number of generated state trajectories|20,000|
|Total state-control data pairs|(4\\times10^5)|
|Training/validation split|80% / 20%|
|Training data pairs|(3.2\\times10^5)|
|Validation data pairs|(8\\times10^4)|
|Data partition strategy|By complete independent trajectories|
|Multi-step training/prediction length|20|
|Early-stopping patience|6 epochs|
|Target validation loss|(5\\times10^{-4})|
|Maximum training iterations|(3\\times10^4)|

## 2\. MPC settings

|Parameter|Value|
|-|-:|
|Sampling time (T\_s)|0.05 s|
|Prediction horizon (N\_p)|10|
|Control horizon (N\_c)|5|
|State weight|(\\mathrm{diag}(0.5,0.5,1))|
|Control-increment weight|(\\mathrm{diag}(0.5,0.5))|
|Relaxation factor weight|0.6|

The MPC is solved in receding-horizon form at every sampling instant. The detailed optimization formulation and constraints follow the manuscript.

## 3\. Final iterative-learning and adaptive settings

|Parameter|Value|
|-|-:|
|Basic ILC learning step (\\alpha\_L)|0.05|
|Adaptive learning gain (\\gamma\_1)|(3.0\\times10^{-5})|
|Adaptive learning gain (\\gamma\_2)|(3.0\\times10^{-5})|
|Adaptive learning gain (\\gamma\_3)|(4.5\\times10^{-5})|
|Numerical denominator regularization floor (B\_{\\min})|0.5|
|DKAIL-MPC cross-iteration error indexing|Spatial index|
|Standalone ILC cross-iteration error indexing|Time index|
|DKAIL-MPC-TimeIndex error indexing|Time index|

For the denominator used in the final numerical adaptive robust compensation calculation, \\\[B\_n^{\\mathrm{ctrl}}=\\max\\left(B\_n^{\\mathrm{raw}},0.5\\right)]\\]. The value 0.5 is a numerical regularization floor. It is not an additional adaptive parameter and does not modify the adaptive-update structure defined by Eqs. (35)–(38) of the manuscript.

## 4\. Iteration-domain update structure

* At (j=0), only MPC feedback is applied.
* For (j\\ge 1), the AILC feedforward term is added to MPC.
* MPC is solved at every sampling instant.
* The three adaptive parameters are updated after completion of one batch according to Eqs. (35)–(37).
* The projection operation in Eq. (38) is applied to keep the adaptive estimates bounded.
* The updated adaptive parameters are used in the next iteration.

For the detailed sequence, see "PSEUDOCODE.md".

## 5\. Comparison and ablation configuration

|Method|Koopman model|Deep state-control joint lifting|MPC|Basic ILC|Adaptive robust terms|Cross-iteration index|
|-|-:|-:|-:|-:|-:|-|
|ILC|No|No|No|Yes|No|Time|
|KMPC|Yes|No|Yes|No|No|N/A|
|DKMPC|No|Yes|Yes|No|No|N/A|
|DKIL-MPC|No|Yes|Yes|Yes|No|Spatial|
|KAIL-MPC|Yes|No|Yes|Yes|Yes|Spatial|
|DKAIL-MPC|No|Yes|Yes|Yes|Yes|Spatial|
|DKAIL-MPC-TimeIndex|No|Yes|Yes|Yes|Yes|Time|

For DKAIL-MPC-TimeIndex, the targeted ablation is the replacement of spatially indexed cross-iteration error transfer by time-indexed error transfer while retaining the remaining principal DKAIL-MPC controller configuration.

## 6\. Released path data

The accompanying file "data/tracking\_data.csv", contains the released annular path numerical data for the reference path, KMPC, KAIL-MPC, and DKAIL-MPC.

The expected principal columns are:
x\_ref
y\_ref
x\_KMPC
y\_KMPC
x\_KAILMPC
y\_KAILMPC
x\_DKAILMPC
y\_DKAILMPC

## 7\. Scope note

This file supports interpretation and verification of the public release. The complete protected engineering configuration, source code, trained model weights, raw training dataset, hardware interfaces, and internal revision/diagnostic files are not part of this public package.

