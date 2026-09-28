# DKAIL-MPC Reproducibility and Verification Materials

This repository contains the non-confidential reproducibility and verification materials accompanying the manuscript: "Iterative learning model predictive control for nonlinear systems based on deep Koopman operator".

The purpose of this repository is to provide sufficient algorithmic, parameter, and numerical information for readers to understand the principal workflow of the proposed method and to independently verify the released annular path tracking results.

This repository is not a release of the complete engineering software used in the project.

## 1\. Scope of the release

The public package contains:

* algorithmic pseudocode for the DKAIL-MPC framework;
* definitions/configurations of the comparison and ablation methods;
* principal model, training, MPC, and iterative learning parameters reported in the manuscript or used in the numerical implementation;
* the annular reference path coordinates;
* the annular path tracking coordinates produced by KMPC, KAIL-MPC, and DKAIL-MPC.

### 2\. Repository structure

DKAIL-MPC-Reproducibility/
├── README.md
├── PSEUDOCODE.md
├── PARAMETERS.md
└── data/
             └── tracking\_data.csv

README.md: Describes the purpose, scope, directory structure, verification procedure, software environment, and release limitations.

PSEUDOCODE.md: Provides the offline Deep Koopman modeling workflow, DKAIL-MPC control procedure, adaptive-parameter update, spatial-indexed error transfer, numerical denominator regularization, and configurations of the seven comparison/ablation methods.

PARAMETERS.md: Summarizes the principal publicly reported modeling, training, MPC, and iterative learning parameters.

data/tracking\_data.csv: Contains the released numerical coordinates for the representative annular path comparison.

The intended principal columns are:
x\_ref
y\_ref
x\_KMPC
y\_KMPC
x\_KAILMPC
y\_KAILMPC
x\_DKAILMPC
y\_DKAILMPC

## 3\. Comparison and ablation methods

|Method|Prediction/control configuration|Cross-iteration learning|
|-|-|-|
|ILC|Standalone ILC|Time index|
|KMPC|Koopman+MPC|None|
|DKMPC|Deep Koopman joint lifting+MPC|None|
|DKIL-MPC|Deep Koopman+MPC+basic ILC|Spatial index|
|KAIL-MPC|Koopman+MPC+adaptive ILC|Spatial index|
|DKAIL-MPC|Deep Koopman+MPC+adaptive ILC|Spatial index|
|DKAIL-MPC-TimeIndex|Same principal DKAIL-MPC modules, with time-indexed error transfer|Time index|

For the precise module-level definitions, see "PSEUDOCODE.md".

## 4\. Software environment

The numerical modeling and control studies reported in the manuscript were carried out in a MATLAB R2024b environment.

The files released here are Markdown and CSV files and therefore do not require MATLAB merely to inspect the released material or recalculate the trajectory-based error quantities.

No executable engineering controller, trained-network file, proprietary model file, or hardware-interface software is included.

## 5\. Reproducibility boundary

The complete engineering implementation forms part of an ongoing collaborative project and contains implementation details subject to confidentiality and intellectual-property restrictions.

Accordingly, the following materials are not included:

* complete MATLAB engineering source code;
* MPC/AILC implementation files;
* Deep Koopman training source code;
* trained Deep Koopman network weights/model files;
* project-protected raw training data;
* hardware and engineering communication interfaces;
* unreleased raw experimental/internal diagnostic data;
* bootstrap resampling files and internal statistical-processing records;
* internal development, tuning, and diagnostic versions used during revision.

The released package is intended to support independent understanding and verification of the principal algorithmic workflow and the released annular path numerical results, rather than complete reconstruction of every simulation or engineering experiment reported in the manuscript.

## 6\. Notes on numerical implementation

Selected implementation details that materially affect the final numerical realization are disclosed in "PSEUDOCODE.md" and "PARAMETERS.md".

In particular:

* basic ILC learning step: \\(\\alpha\_L=0.05\\);
* adaptive learning gains: \\\[\\gamma\_1,\\gamma\_2,\\gamma\_3]^T=\[3.0\\times10^{-5},,3.0\\times10^{-5},,4.5\\times10^{-5}]^T\\];
* numerical denominator regularization floor: (B\_{\\min}=0.5);
* DKAIL-MPC uses spatial index cross-iteration error transfer;
* standalone ILC uses time index error transfer;
* DKAIL-MPC-TimeIndex uses time index error transfer as the targeted indexing ablation.

The denominator floor is a numerical regularization setting, not an additional adaptive learning parameter.

