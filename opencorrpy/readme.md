opencorrpy - Python interface to OpenCorr： An open source C++ library for digital image correlation

WARNING: CURENT VERSION IS IN EARLY DEVELOPMENT PHASE, DOESN'T HAVE STABLE INTERFACE AND IT IS NOT READY FOR TESTING!

# 1. Introduction 
In addition to files in the original OpenCorr repostitory (https://github.com/vincentjzy/OpenCorr), this repository (https://github.com/pprebeg/OpenCorr) in branch pybind11 contains pybind11 interface for some predifined 3D DIC workflows. The interface is defined in opencorrpy/opencorrpy.cpp. For extension of currently defined workflows or definition of some other workflows, pybind11 interface have to be changed. As a consequence, Python users options for solving a problems with OpenCorr are limited to currently implemented workflows.  

# 2. Building the library 
In addition to opencorrpy/opencorrpy.cpp, the branch pybind11 contains Visual Studion 2019 C++ project  opencorrpy/opencorrpy.vcxproj. The project file enables generation of opencorrpy.pyd library that can be used directly inPython.
The folowing procedure is tested in Visual Studio 2019 on Windows10 OS, however it sholud work in Windows11 OS without any modification.  

Linux users should prepare project file for *pyd library from scratch using opencorrpy/opencorrpy.cpp and the rest of the OpenCorr source files.  

1. Clone/download repository from pprebeg/OpenCorr  
2. Add the additional libraries and environment variable  
For the definition of python/pybind11 inclides/libraries OCRRPY_ENV_DIR environment variable is used. Add OCRRPY_ENV_DIR to environment variables, where the value is path to Python environment that will be used for build and execution. Python environment must have pybind11 installed. Alternative is to build/install pybind11 from source code and change the location of relevant include directory in project properties.
The project file is defined with the folowing intended folders structure
.../OpenCorrSolutionFolder(name,name is not relevant)  
- OpenCorr(root folder of repository, name is not relevant)  
- Eigen3  
- opencorrGPU  
- opencv  
- fftw3  
- nanoflann  
- *.sln (name is not relevant)  

The additional libraries (opencv,fftw3, ...) can be installed/copied folowing the procedure defined in 1_Get_started.md. The root folder of each library should remain the same as given in the intended folders structure. Othervise project file needs to be modified.  

3. Open the project file opencorrpy.vcxproj in Visual Studion 2019 (or later) and save solution file to OpenCorrSolutionFolder.  

4. If necessary, modify project properties.  
Users that have complitly folowed step 2, no changes in project file is not necessary.
For users that will not define environment variable OCRRPY_ENV_DIR and/or folder structure of the project as given in step 2,  properties will have to be changed. For help in this process, the list of project properties that depends on environment variable OCRRPY_ENV_DIR and/or folder structure is given in sequel:
C/C++/ General/Additional Include Directories:  $(OCRRPY_ENV_DIR)\include;$(OCRRPY_ENV_DIR)\Lib\site-packages\pybind11\include  
Linker/General/Additional Library Directories: $(OCRRPY_ENV_DIR)\libs  
C/C++/ General/Additional Include Directories: $(SolutionDir)OpenCorr\src  
C/C++/ General/Additional #using Directories: $(SolutionDir)OpenCorr\src  
Linker/Input/AdditionalDependencies: libfftw3-3.lib;libfftw3f-3.lib;libfftw3l-3.lib;opencv_world460.lib;OpenCorrGPULib.lib;  
VC++ Directories/ Include Directories: $(SolutionDir)Eigen3;$(SolutionDir)Eigen3\Eigen;$(SolutionDir)fftw3\include;$(SolutionDir)opencv\include;$(SolutionDir)nanoflann  
VC++ Directories/ Library Directories: $(SolutionDir)fftw3\lib;$(SolutionDir)opencv\lib;$(SolutionDir)opencorrGPU\lib  

5. Build the project  
On secesfull build, opencorrpy.pyd will be generated. The location of generated file for Release/x64 configuration is OpenCorrSolutionFolder/bin. In order to use the generated library, dll files of Additional libraries (libfftw3-3.dll, libfftw3f-3.dll, libfftw3l-3.dll, OpenCorrGPULib.dll) should be in OS path or the dll files have to be added to the same folder where opencorrpy.pyd is.  

# 3.  Testing the library  
The library can be tested using files test_3d_dic_epipolar_sift.py, test_3d_dic_strain.py. The test files presumes that the opencorrpy.pyd is located in  OpenCorrSolutionFolder/bin. The results should be equivalent to the results obtained by original cpp versions of those examples.  




 