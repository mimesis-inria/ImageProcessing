# README #

### What is this repository for? ###

* The ImageProcessing plugin is a plugin for the simulation framework SOFA [link](www.sofa-framework.org). Its purpose is to provide general purpose Computer vision featuresto Sofa. More specifically, its goal is to enable the use of SOFA simulations in augmented reality, and virtual reality applications.
The ImageProcessing plugin depends on the SofaCV Base plugin. It is recommended to use the DataAcquisition plugin too, to easily load images and videos from your filesystem, from the network etc.
The ImageProcessing plugin doesn't have the ambition to provide cutting-edge computer vision algorithms, but instead, wraps OpenCV's features while taking advantage of SOFA's Component-based API. This plugin also provides useful Camera components, providing on-the-fly conversions from/to OpenGL & OpenCV's camera calibration parameters. (calibrating cameras, modifying both intrinsic and extrinsic camera parameters in OpenGL)

* Version 2.0

### To setup the ImageProcessing plugin: ###

1 Install Third party dependencies
* The ImageProcessing plugin depends on an API plugin called SofaCV, available here: [gitlab.inria.fr/mimesis/ComputerVision/SofaCV](gitlab.inria.fr/mimesis/ComputerVision/SofaCV). Documentation on how to install it is available in the repository's README.md file.
* This plugin also relies on some optional dependencies:
    * __opencv_contrib__, the non-free module from OpenCV, containing all non-BSD licensed code (such as the SIFT feature detector for instance): [github.com/opencv/opencv_contrib.git](github.com/opencv/opencv_contrib)
    * __SofaQtQuick__, the new GUI for sofa, allows for custom widgets for components. this is heavily used to debug/tune image processing filters, display camera views, and custom widgets that require additional controls (manual segmentation tools for instance). To use the new GUI's functionalities, the SofaQtQuick plugin needs to be activated and compiled with SOFA. Checkout SofaQtQuick's documentation for more information (https://github.com/sofa-framework/SofaQtQuick)

2 Install SOFA with ImageProcessing:
* Download Sofa-framework from Github [github.com/sofa-framework/sofa.git](github.com/sofa-framework/sofa)
* cd ${SOFA_SRC_DIR}; mkdir build ; cd build ; cmake-gui ${SOFA_SRC_DIR}
* in CMAKE's gui, search for the SOFA_EXTERNAL_DIRECTORIES, and set this variable to the path to the SofaCV repository
* Configuration
    * If you are not interested in any of the features unlocked by the optional dependencies, all you have to do now is generating your CMake project, and compiling your code. example scenes can be found in SofaOR/examples
    * If you are interested in using OpenCV's 3rd party features with SOFA, you might be interested in activating OpenCV's contrib wrapping through CMake:
        * To enable opencv_contrib features: __SOFAOR_ENABLE_OPENCV_CONTRIB__. Don't forget to compile OpenCV with the contrib module

### Contribution guidelines ###

* __Contributing to this plugin:__
    ImageProcessing has 2 main branches, __master__ and __develop__. both branches are protected, and you need to create pull (merge) requests to contribute to the codebase. merge requests are made against develop, except for __FAST-MERGE__ labelled PRs that are merged against both master and develop. merged PRs in develop are merged back into master once they've proven themselves stable enough

* __Coding styles:__
    SofaCV is developped using a (slightly) modified Google / gnu codestyle. To contribute, please try to respect this indentation. More information on the coding style can be found in the SofaCV's README.md file.

### Who do I talk to? ###

* Repo owner or admin: bruno.josue.marques@inria.fr
* Other community or team contact: mimesis@inria.fr
