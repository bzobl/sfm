PROJECT = sfm

CC = gcc
CFLAGS = 
# CFLAGS for all configuration. cannot be overwritten by environment
override CFLAGS += -Wall -O3 -std=c++11

RM = rm -f

CUDA_TOOLBOX_DIR = /opt/cuda-6.5
CUDA_CC = $(CUDA_TOOLBOX_DIR)/bin/nvcc
CUDA_CFLAGS =
override CUDA_CFLAGS += -arch=sm_20 -ccbin $(CC) --relocatable-device-code=true -std=c++11

C_INCL = ./include 											\
				 $(CUDA_TOOLBOX_DIR)/include		\
				 /opt/opencv3/include						\
				 /usr/include/eigen3 						\
				 /usr/include/vtk-5.8 					\
				 /opt/pcl/include/pcl-1.8
INCLUDES = $(addprefix -I, $(C_INCL))

C_LIB_DIRS = /opt/opencv3/lib						\
						 /opt/pcl/lib								\
						 $(CUDA_TOOLBOX_DIR)/lib	  \
						 $(CUDA_TOOLBOX_DIR)/lib64
LIB_DIRS = $(addprefix -L, $(C_LIB_DIRS))

OPENCV_LIBS = core highgui imgproc imgcodecs videoio video 	\
							features2d xfeatures2d calib3d 								\
							cuda cudaoptflow cudafeatures2d
PCL_LIBS = common visualization io filters
BOOST_LIBS = system
VTK_LIBS = Common Views Filtering Rendering
C_LIB = $(addprefix opencv_, $(OPENCV_LIBS)) \
				$(addprefix pcl_, $(PCL_LIBS)) 			 \
				$(addprefix boost_, $(BOOST_LIBS))   \
				$(addprefix vtk, $(VTK_LIBS)) 			 \
				pthread 														 \
				cudart cublas

LIBS = $(addprefix -l, $(C_LIB))

CPP_SRC  = main.cpp										\
					 frame.cpp									\
					 livestream.cpp 						\
					 alpha-image.cpp 						\
					 ViewCombination.cpp				\
					 OpenCVPoseEstimation.cpp   \
					 OpenCVFeatureDetector.cpp  \
					 PointCloud.cpp							\
					 IPoseEstimation.cpp				\
					 GPUIterativeLinearLS.cpp		\
					 GPUSurf.cpp 								\
					 RubikManualFeatures.cpp 		\
					 ITriangulation.cpp 				\
					 OpticalFlowFeatures.cpp 		\
					 Camera.cpp 								\
					 tests.cpp

CPP_H    = $(wildcard $(C_INCL)/*.h)

CUDA_SRC = triangulation.cu

CPP_OBJS = $(CPP_SRC:%.cpp=%.o)
CUDA_OBJS = $(CUDA_SRC:%.cu=%.o)

DEPS = $(CPP_OBJS:%.o=%.d) \
			 $(CUDA_OBJS:%.o=%.d)

all: $(PROJECT)

-include $(DEPS)

%.o: %.cpp
	@echo 'Building file: $<'
	$(CUDA_CC) $(CUDA_CFLAGS) -Xcompiler "$(CFLAGS) $(INCLUDES)" -c -o "$@" "$<"
	@echo 'Building dependencies: $<'
	$(CC) -MM $(CFLAGS) $(INCLUDES) "$<" > $*.d
	@echo 'Finished building $<'
	@echo ' '

%.o: %.cu
	@echo 'Building cuda file: $<'
	$(CUDA_CC) $(CUDA_CFLAGS) $(INCLUDES) -c -o "$@" "$<"
	#@echo 'Building dependencies: $<'
	#$(CUDA_CC) $(CUDA_CFLAGS) -Xcompiler "-MM $(CFLAGS) $(INCLUDES)" "$<" > $*.d
	@echo 'Finished building $<'
	@echo ' '

$(PROJECT): $(CPP_OBJS) $(CUDA_OBJS)
	@echo 'Linking file: $@'
	$(CUDA_CC) -o $@ $(CUDA_CFLAGS) -Xcompiler "$(CFLAGS) $(INCLUDES) $(LIB_DIRS)" $(CPP_OBJS) $(CUDA_OBJS) $(LIBS)
	@echo 'Finished building $@'
	@echo ' '

schroot:
	schroot -c exp -- make PREFIX="" $(PROJECT)

tegra:
	make CUDA_TOOLBOX_DIR=/usr/local/cuda-6.5

debug:
	make CFLAGS="-DPRINT" CUDA_CFLAGS="-DPRINT"

const-mem:
	make CFLAGS="-DUSE_CONSTANT_MEMORY" CUDA_CFLAGS="-DUSE_CONSTANT_MEMORY"

version:
	$(CC) --version

clean:
	$(RM) $(CPP_OBJS) $(CUDA_OBJS) $(DEPS) $(PROJECT) .depend
