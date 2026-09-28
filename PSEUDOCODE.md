# DKAIL-MPC Algorithm Pseudocode

## 1\.  Purpose

This document provides the algorithmic pseudocode and key numerical implementation notes for the proposed Deep Koopman operator-based adaptive iterative learning model predictive control (DKAIL-MPC) method. Its purpose is to improve the transparency and verifiability of the reported method and the released numerical results.

The method mainly consists of:

1. a Deep Koopman state–control joint-lifting prediction model;
2. an MPC feedback controller operating in the time domain;
3. an AILC feedforward controller operating in the iteration domain;
4. a path arc length based spatial indexing mechanism for cross-iteration error transfer.

The complete engineering source code, trained model weights, project-protected raw training data, low-level solver implementation, and engineering interfaces are not included in the public materials.

\---

## 2\. Main Notation

|Symbol|Meaning|
|-|-|
|(j)|Iteration index|
|(k)|Discrete time index within one iteration|
|(N)|Number of samples in one trajectory|
|(J)|Maximum number of iterations|
|(\\mathbf{x}\_j(k))|System state|
|(\\mathbf{r}(k))|Reference state|
|(\\mathbf{e}\_j(k))|Tracking error|
|(s\_j(k))|Path arc length spatial index|
|(\\mathbf{z}\_j(k))|Deep Koopman lifted state|
|(\\mathbf{v}^{\\mathrm{MPC}}\_j(k))|MPC feedback control input|
|(\\mathbf{v}^{\\mathrm{ILC}}\_j(k))|AILC feedforward control input|
|(\\mathbf{v}\_j(k))|Total control input|
|(N\_p,N\_c)|MPC prediction and control horizons|

For the complete DKAIL-MPC method, the control input is \\mathbf v\_j(k)=\\mathbf v^{\\mathrm{MPC}}\_j(k)+\\mathbf v^{\\mathrm{ILC}}\_j(k).

At the first iteration (j=0), no previous-iteration error information is available; therefore, only MPC is applied.

\---

## 3\. Algorithm A: Offline Construction of the Deep Koopman State–Control Joint-Lifting Model

Input:
Training samples {x(k), u(k), x(k+1)}
State-lifting network configuration
Control-lifting network configuration
Training settings
Output:
State-lifting mapping φx(·)
Control-lifting mapping φu(·)
Trained Deep Koopman prediction model
1: Initialize the state-lifting and control-lifting networks.
2: For each training sample: obtain x(k), u(k), and x(k+1); lift the current state z(k)=φx(x(k)); lift the control input v(k)=φu(u(k)); lift the next state z(k+1)=φx(x(k+1)).
3: Construct the approximately linear evolution relation in the joint lifted state–control space.
4: Optimize the lifting networks and Koopman-model parameters according to the training objective defined in the manuscript.
5: After training, fix φx(·), φu(·), and the Deep Koopman prediction-model parameters.
6: Use the fixed trained model for online MPC prediction.

**Note:** This algorithm describes the offline modeling workflow reported in the manuscript. The public materials do not include the training source code, trained model weights, or project-protected raw training data.

\---

## 4\. Algorithm B: DKAIL-MPC Repetitive Trajectory-Tracking Control

Input:
Reference trajectory r
Trained Deep Koopman prediction model
Prediction horizon Np
Control horizon Nc
MPC weighting matrices and constraints
Basic ILC learning step αL
Adaptive learning gains γ1, γ2, γ3
Adaptive-parameter projection bounds
Number of samples N
Maximum number of iterations J
Initialize:
Set iteration index j=0.
Initialize the adaptive parameters.
Initialize the spatial-indexed error memory.
1: while j<J
2:     Set k=0.
3:     Initialize the state, control, and error records for iteration j.
4:     while k<N

A. State and tracking-error acquisition

5:         Acquire the current system state x\_j(k).
6:         Obtain the corresponding reference state r(k).
7:         Calculate the tracking error e\_j(k) according to the error definition in the manuscript.

B. Spatial indexing

8:         Determine the current location relative to the reference path.
9:         Determine the corresponding path arc length spatial index s\_j(k).
10:        Use s\_j(k), rather than the time index k, to align the cross-iteration error information used by DKAIL-MPC.

C. Deep Koopman prediction and MPC

11:        Calculate the lifted state z\_j(k)=φx(x\_j(k)).
12:        Construct the prediction over the MPC horizon using the trained Deep Koopman model.
13:        Form the finite-horizon constrained MPC problem using the current lifted state, future reference, Np, Nc, weighting matrices, and constraints.
14:        Solve the MPC optimization problem.
15:        Take the first element of the optimal sequence as v\_MPC,j(k).

D. First iteration

16:        if j==0
17:            Set v\_j(k)=v\_MPC,j(k).

E. Subsequent iterations

18:        else
19:            Retrieve the previous-iteration tracking-error information corresponding to the current spatial location s\_j(k).
20:            Calculate the basic iterative-learning term v\_1,j(k) using the learning step αL=0.05.
21:            Calculate the three adaptive robust compensation terms according to the control laws in the manuscript: v\_2,j(k), v\_3,j(k), v\_4,j(k).
22:            Construct the AILC feedforward term v\_ILC,j(k)=v\_1,j(k)+v\_2,j(k)+v\_3,j(k)+v\_4,j(k).
23:            Combine MPC feedback and AILC feedforward v\_j(k)=v\_MPC,j(k)+v\_ILC,j(k).
24:        end if

F. Control execution and data recording

25:        Enforce the prescribed control constraints.
26:        Apply v\_j(k) to the nonlinear system.
27:        Record the resulting state and tracking error.
28:        Store the error information according to spatial index s\_j(k) for the next iteration.
29:        k=k+1.
30:    end while

G. Iteration-domain adaptive-parameter update

31:    Using the data collected over iteration j, calculate the candidate updates of the three adaptive parameters according to Eqs. (35)–(37) of the manuscript.
32:    Use the adaptive learning gains γ = \[3.0×10^(-5), 3.0×10^(-5), 4.5×10^(-5)]^T.
33:    Apply the projection operator in Eq. (38) to each candidate parameter: θ\_hat\_(j+1)=Proj\_\[θ\_lower, θ\_upper](θ\_hat\_candidate), where Proj\_\[a,b](q) = max(a, min(b,q)).
34:    Store the projected adaptive parameters for iteration j+1.
35:    Retain the spatial-indexed error memory generated in iteration j for cross-iteration learning.
36:    j=j+1.
37: end while
Output:
State trajectories
Control inputs
Tracking errors
Spatial-indexed error memory
Adaptive-parameter histories

**Note:** MPC is solved in a receding-horizon manner at each sampling instant (k). In subsequent iterations, the AILC compensation uses error information from the preceding iteration. The three adaptive parameters are updated after completion of an iteration according to Eqs. (35)–(37) of the manuscript and are bounded using the projection operator in Eq. (38). Thus, the current iteration uses the adaptive parameters assigned to that batch, and the updated parameters are used in the next iteration.

\---

## 5\. Numerical Denominator Regularization

In the numerical implementation, a lower-bound regularization is applied to the relevant quantity when it is used as a denominator in the adaptive robust compensation calculation:

\\\[B\_n^{\\mathrm{ctrl}}=\\max\\left(B\_n^{\\mathrm{raw}},B\_{\\min}\\right),B\_{\\min}=0.5\\]

The implementation logic is:

1: Compute the original quantity: rawBn = Bn\_raw.
2: Preserve rawBn wherever the unregularized quantity is required by the algorithm.
3: For the denominator used in the numerical robust compensation calculation, set: ctrlBn = max(rawBn, 0.5).
4: Use ctrlBn only in the calculation for which the denominator regularization is required.

Here, (B\_{\\min}=0.5) is a numerical regularization floor used to avoid numerical amplification caused by an excessively small denominator. It is not an additional adaptive learning parameter and does not modify the adaptive update structure defined by Eqs. (35)–(38) of the manuscript.

\---

## 6\. Algorithm C: Batch Update of the Adaptive Parameters

Input:
Data collected during iteration j
Current adaptive parameters
Learning gains γ1, γ2, γ3
Projection bounds
1: Use the required data collected over iteration j.
2: Calculate the candidate update of adaptive parameter 1 according to Eq. (35).
3: Calculate the candidate update of adaptive parameter 2 according to Eq. (36).
4: Calculate the candidate update of adaptive parameter 3 according to Eq. (37).
5: Apply the projection operator in Eq. (38) to each candidate parameter.
6: Store the projected parameters.
7: Use the projected parameters during iteration j+1.

**Note:** The three adaptive parameters are not relearned at every sampling instant within the same iteration. They are updated along the iteration axis after completion of one trajectory execution.

\---

## 7\. Algorithm D: Spatial-Indexed Cross-Iteration Error Transfer

Input:
Current system/path location
Reference path
Previous-iteration spatial error memory
1: Determine the current location relative to the reference path.
2: Determine the corresponding path-arc-length coordinate/index s.
3: Use s as the cross-iteration error-memory index.
4: Retrieve the previous-iteration tracking-error information associated with the corresponding spatial location.
5: Use the retrieved error information in the ILC/AILC calculation.
For the complete DKAIL-MPC method, cross-iteration error transfer can be summarized as \[\\mathbf e\_{j-1}(s) \\longrightarrow \\mathbf v^{\\mathrm{ILC}}\_j(s)]

That is, error information from the preceding iteration is transferred according to path location rather than simply according to the same time index.

\---

## 8\. Configuration of Comparison and Ablation Methods

### 8.1 ILC

The standalone ILC baseline uses a time index. Its iterative learning update uses error information from the same time index of the preceding iteration. MPC, Koopman/Deep Koopman prediction, and the three adaptive robust compensation terms are disabled.

Cross-iteration error index: time index k
MPC: disabled
Koopman/Deep Koopman prediction: disabled
Adaptive robust compensation: disabled

### 8.2 KMPC

Koopman prediction model: enabled
Deep state–control joint lifting: disabled
MPC: enabled
ILC/AILC: disabled

### 8.3 DKMPC

Deep state-control joint lifting: enabled
MPC: enabled
ILC/AILC: disabled

### 8.4 DKIL-MPC

Deep state–control joint lifting: enabled
MPC: enabled
Basic iterative learning: enabled
Adaptive robust compensation: disabled
Cross-iteration error alignment: spatial index

DKIL-MPC retains the basic iterative-learning term (\\mathbf v\_{1,j}) while disabling the three adaptive robust compensation terms: \\mathbf v\_{2,j}=\\mathbf v\_{3,j}=\\mathbf v\_{4,j}=0

### 8.5 KAIL-MPC

Koopman prediction model: enabled
Deep state–control joint lifting: disabled
MPC: enabled
Basic iterative learning: enabled
Adaptive robust compensation: enabled
Cross-iteration error alignment: spatial index

### 8.6 DKAIL-MPC — Complete Proposed Method

Deep state–control joint lifting: enabled
MPC: enabled
Basic iterative learning: enabled
Adaptive robust compensation: enabled
Cross-iteration error alignment: spatial index

The total control input is \[\\mathbf v\_j=\\mathbf v\_j^{\\mathrm{MPC}}+\\mathbf v\_{1,j}+\\mathbf v\_{2,j}+\\mathbf v\_{3,j}+\\mathbf v\_{4,j}]

### 8.7 DKAIL-MPC-TimeIndex

DKAIL-MPC-TimeIndex retains the Deep Koopman model, MPC, basic ILC, adaptive robust compensation, learning parameters, and control constraints of the complete DKAIL-MPC method. The targeted change is to replace path arc length-based spatial error alignment with time-indexed cross-iteration error alignment.

DKAIL-MPC: previous-iteration error index=spatial index s
DKAIL-MPC-TimeIndex: previous-iteration error index=time index k

\---

## 9\. Key Released Implementation Settings

|Setting|Value|
|-|-:|
|Basic ILC learning step (\\alpha\_L)|0.05|
|Adaptive learning gain (\\gamma\_1)|(3.0\\times10^{-5})|
|Adaptive learning gain (\\gamma\_2)|(3.0\\times10^{-5})|
|Adaptive learning gain (\\gamma\_3)|(4.5\\times10^{-5})|
|Numerical denominator regularization floor (B\_{\\min})|0.5|
|DKAIL-MPC cross-iteration indexing|spatial|
|Standalone ILC cross-iteration indexing|time|
|DKAIL-MPC-TimeIndex cross-iteration indexing|time|

The remaining Deep Koopman model parameters, MPC weights, prediction/control horizons, and constraints follow the settings reported in the manuscript.

\---

## 10\. Released Materials and Verification Scope

The released materials include:

* the coordinates of the annular reference path;
* the tracking coordinates produced by KMPC;
* the tracking coordinates produced by KAIL-MPC;
* the tracking coordinates produced by DKAIL-MPC;
* the algorithmic pseudocode and key numerical implementation settings documented here;
* the principal model and control parameters already reported in the manuscript.

These materials are intended to support independent understanding of the principal algorithmic workflow and verification of the released annular path results. They do not constitute a release of the complete engineering software and should not be interpreted as enabling complete reconstruction of every experiment reported in the manuscript.

The complete engineering source code, trained model weights, project protected raw training data, engineering interfaces, unreleased raw experimental data, and other implementation materials subject to intellectual-property or project-confidentiality restrictions are not included in the public package.

