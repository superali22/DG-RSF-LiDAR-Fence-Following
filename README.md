Overview

This repository provides a research implementation and supplementary simulation resources for a dynamic-gated fence-following method developed for autonomous feed-pushing robots in dairy farming.

Autonomous feed-pushing robots rely on reliable fence-boundary references to navigate along feed alleys. However, dust interference, sparse structural returns, and temporary occlusions can degrade 2D LiDAR observations and cause unstable boundary estimation.

The proposed method uses dynamic gating to adapt candidate boundary observations according to boundary-state reliability and maintains temporal consistency to provide a stable fence reference for offset-path generation.

The repository is intended to support algorithm inspection, methodological demonstration, and supplementary evaluation under controlled simulation conditions.

Key Features

Dynamic-gated boundary estimation: Adapts candidate boundary observations according to boundary-state reliability.

Temporal boundary-state maintenance: Preserves boundary-reference continuity for fence-following navigation.

Simulated feed-alley dataset: Provides LiDAR data collected in a purpose-built simulated feed alley.

Dust-interference simulation: Includes code for simulating dust-related interference in LiDAR observations.

Reproducibility resources: Supports examination and testing of the released research implementation.

Repository Contents

The repository contains the following types of resources:

Research implementation of the proposed boundary-estimation method.

LiDAR data collected in a simulated feed-alley environment.

Code for simulating dust-related LiDAR interference.

Supporting configuration files, scripts, and documentation, where provided.

Please refer to the actual files and accompanying instructions in this repository for the available implementation components and usage details.

Data Description

Simulated Dataset

The publicly released dataset was collected in a purpose-built simulated feed-alley environment. It is smaller than the original field dataset and is intended for controlled testing and supplementary evaluation.

The accompanying dust-interference simulation code allows users to examine the behavior of the released implementation under simulated degraded LiDAR observations.

The simulated data do not reproduce every characteristic of real dairy-farm environments and should not be considered a replacement for field data.

Original Field Dataset

The original field benchmark contains 120,000 LiDAR frames collected at two commercial dairy farms in Yinchuan, China:

60,000 frames collected during normal operation.

60,000 frames collected during naturally occurring dust exposure periods.

Reference fence boundaries were established through automatic boundary initialization followed by manual verification and correction.

The original field recordings and associated reference annotations are not publicly available because they contain commercially sensitive information about the participating farms, including facility and environmental characteristics.

Implementation Scope

The publicly released implementation is a separate research version that closely follows the original method and preserves its core algorithmic functionality. It may differ from the complete implementation used in the field experiments.

The simulated dataset and dust-interference code support supplementary examination and evaluation of the method under controlled conditions. The real-world performance reported in the associated research article is based on the original field experiments, not on the simulated dataset alone.

Installation and Usage

Please refer to the source files, configuration files, and scripts included in the repository for implementation-specific requirements.

Before running the code, users should verify the required software dependencies, input data format, coordinate conventions, and configuration parameters.

Example commands and detailed instructions should be added here once the released implementation has been verified to run from a clean installation.

Reproducibility and Limitations

This repository is intended to facilitate methodological transparency and supplementary reproducibility.

The following limitations should be considered:

The simulated dataset is smaller than the original field benchmark.

Simulated dust interference may not capture all characteristics of naturally occurring dust in commercial dairy farms.

The released research implementation may differ from the exact software version used in the original field experiments.

The public resources do not include the original 120,000-frame field benchmark or its complete reference annotations.

Accordingly, the released resources supplement, rather than replace, the real-world validation reported in the associated study.

Associated Publication

Title: [Dynamic-Gated Fence Following for 2D LiDAR-Based Feed-Pushing Robots]

Journal: Computers and Electronics in Agriculture

Manuscript ID: COMPAG-D-26-08350

The final bibliographic information will be added after publication.

Citation

If you use this repository in your research, please cite the associated publication once its bibliographic information becomes available.

Contact

For questions concerning the released research implementation and simulated dataset, please use the contact information provided in the associated publication.

License and Copyright

The software copyright and authorization status of the released materials should be verified before public distribution. The applicable license and reuse permissions will be specified once the rights holder has authorized the release.

The availability of this repository does not imply that the original field dataset or the complete field implementation is publicly available.
