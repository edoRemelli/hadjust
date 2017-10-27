# hadjust
Low-Dimensionality Calibration through Local Anisotropic Scaling for Robust Hand Model Personalization (ICCV17)

# abstract
We present a robust algorithm for personalizing a sphere- mesh tracking model to a user from a collection of depth measurements. Our core contribution is to demonstrate how simple geometric reasoning can be exploited to build a shape-space, and how its performance is comparable to shape-spaces constructed from datasets of carefully calibrated models. We achieve this goal by first re-parameterizing the geometry of the tracking template, and introducing a multi-stage calibration optimization. Our novel parameterization decouples the degrees of freedom for pose and shape, resulting in improved convergence properties. Our analytically differentiable multi-stage calibration pipeline optimizes for the model in the natural low-dimensional space of local anisotropic scalings, leading to an effective solution that can be easily embedded in other tracking/calibration algorithms. Compared to existing sphere-mesh calibration algorithms, quantitative experiments assess our algorithm possesses a larger convergence basin, and our personalized models allows to perform motion tracking with superior accuracy.

# usage
Put the depth measurements in folder data, similarly to what is done for USER1.
To run the calibration and save the calibrated sphere-mesh model to file run respectively main/MAIN_CALIBRATE.m and main/WRITE_CALIBRATED_MODEL_TO_FILE.m
Please note that .mex files are built for windows usage, you will have to re-build them to use the code on a different OS.

# paper
[a link](http://lgg.epfl.ch/publications/2017/LocalAnisotropicScaling/paper.pdf)
[a link](http://lgg.epfl.ch/publications/2017/HOnline/paper.pdf)
[a link](http://lgg.epfl.ch/publications/2016/HModel/paper.pdf)
