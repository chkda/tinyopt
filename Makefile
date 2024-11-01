CXX = clang++
NVCC = nvcc
CUDA_PATH = /usr/local/cuda

# Get OpenCV flags and libraries using pkg-config
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS := $(shell pkg-config --libs opencv4)

# Compiler flags
CXXFLAGS = -std=c++17 \
           -Wall \
           -Wextra \
           -I./include \
           -I$(CUDA_PATH)/include \
           $(OPENCV_CFLAGS)

# CUDA flags
NVCCFLAGS = -std=c++17 \
            -I./include \
            -I$(CUDA_PATH)/include \
            $(OPENCV_CFLAGS) \
            -Xcompiler -Wall

# Directories
BUILD_DIR = build
CPU_SRC = src
CUDA_SRC = src/cuda

# Source files
CPU_SOURCES = $(wildcard $(CPU_SRC)/*.cpp)
CUDA_SOURCES = $(wildcard $(CUDA_SRC)/*.cu)
CPU_OBJECTS = $(CPU_SOURCES:$(CPU_SRC)/%.cpp=$(BUILD_DIR)/%.o)
CUDA_OBJECTS = $(CUDA_SOURCES:$(CUDA_SRC)/%.cu=$(BUILD_DIR)/cuda/%.o)  # Changed this line

# Libraries
LIBS = -L$(CUDA_PATH)/lib64 -lcudart -lcuda $(OPENCV_LIBS)

# Target executable
TARGET = tinyopt

# Default target
all: directories $(TARGET)

# Create build directory
directories:
	mkdir -p $(BUILD_DIR)/cuda  # Added cuda subdirectory

# Link the final executable
$(TARGET): $(CPU_OBJECTS) $(CUDA_OBJECTS) main.cpp
	$(CXX) $(CXXFLAGS) main.cpp $(CPU_OBJECTS) $(CUDA_OBJECTS) $(LIBS) -o $@

# Compile CPU source files
$(BUILD_DIR)/%.o: $(CPU_SRC)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile CUDA source files
$(BUILD_DIR)/cuda/%.o: $(CUDA_SRC)/%.cu  # Changed this line
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# For debugging
print-%:
	@echo $* = $($*)

.PHONY: all clean directories print-%