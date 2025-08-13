# Analysis of Centrifugal Clutches in Two-speed Automatic Transmissions with Multilayer Perceptron Neural Network-based Engagement Prediction

Bo-Yi Lin* and Kai-Chun Lin*, “Analysis of Centrifugal Clutches in Two-Speed Automatic Transmissions
with Multilayer Perceptron Neural Network-Based Engagement Prediction”, IJCAST, (2025). [paper](https://www.inderscienceonline.com/doi/abs/10.1504/IJCAST.2025.147074)

## Abstract
Numerical analysis of centrifugal clutch systems integrated with a two-speed automatic transmission is shown in this paper. Various clutch configurations and their effects on the dynamics of the considered transmission have been examined. Based on these configurations, torque transfer, upshifting, and downshifting behaviors under various conditions are discussed. This paper presents a Deep Neural Network (DNN) model for clutch engagements, whose parameters are spring preload and shoe mass. In this paper, a computationally efficient alternative to these complex simulations for the modeling is presented. Deep learning and numerical modeling further help in the critical insights required for improvement in the design parameters, performance, and efficiency of the clutch-transmission system.

## Introduction
This project focuses on the analysis of centrifugal clutch systems within two-speed automatic transmissions. It explores various clutch configurations and their impact on transmission dynamics, including torque transfer, upshifting, and downshifting. A key contribution is the development of a Deep Neural Network (DNN) model to predict clutch engagement based on parameters like spring preload and shoe mass. This approach offers a computationally efficient alternative to traditional complex simulations, aiding in the design and optimization of clutch-transmission systems.

## Repository Structure
This repository contains the following key files and directories:
- `README.md`: This file, providing an overview of the project.
- `centrifugal_clutch.m`: MATLAB script for the numerical analysis of the centrifugal clutch.
- `config_b_DNN_engagement_prediction.py`: Python script for predicting clutch engagement using the trained DNN model for configuration b.
- `config_b_DNN_model.h5`: The trained HDF5 model file for the DNN for configuration b.
- `config_b_DNN_training.py`: Python script for training the DNN model for configuration b.
- `config_b_engagement_speed.m`: MATLAB script related to engagement speed for configuration b.
- `predict_result_cofig_b.xlsx`: Excel file containing prediction results for configuration b.
- `result_cofig_b.xlsx`: Excel file containing results for configuration b.

## Citation
If you use this work or code in your research, please cite the paper:

```
Bo-Yi Lin* and Kai Chun Lin*. “Analysis of Centrifugal Clutches in Two-Speed Automatic Transmissions
with Deep Learning-Based Engagement Prediction”. arXiv preprint arXiv:2409.09755 (2024).
```
Or in BibTeX format:
```bibtex
@article{lin2024analysis,
  title={Analysis of Centrifugal Clutches in Two-Speed Automatic Transmissions with Deep Learning-Based Engagement Prediction},
  author={Lin, Bo-Yi and Lin, Kai Chun},
  journal={arXiv preprint arXiv:2409.09755},
  year={2024}
}
```
