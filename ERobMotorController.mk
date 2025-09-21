# Makefile for ERobMotorController
# Simple compilation without CMake

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -fPIC
INCLUDES = -I./soem -I./osal -I./osal/linux -I./oshw/linux
LIBS = -lpthread -lrt
SOEM_LIB = ./build/libsoem.so

# Source files
CONTROLLER_SOURCES = ERobMotorController.cpp
EXAMPLE_SOURCES = motor_controller_example.cpp

# Output files
CONTROLLER_LIB = libERobMotorController.so
EXAMPLE_BIN = motor_controller_example

# Default target
all: $(CONTROLLER_LIB) $(EXAMPLE_BIN)

# Build motor controller library
$(CONTROLLER_LIB): $(CONTROLLER_SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -shared -o $@ $^ $(SOEM_LIB) $(LIBS)

# Build example program
$(EXAMPLE_BIN): $(EXAMPLE_SOURCES) $(CONTROLLER_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lERobMotorController $(SOEM_LIB) $(LIBS)

# Clean build artifacts
clean:
	rm -f $(CONTROLLER_LIB) $(EXAMPLE_BIN)

# Install (requires sudo)
install: $(CONTROLLER_LIB)
	sudo cp $(CONTROLLER_LIB) /usr/local/lib/
	sudo cp ERobMotorController.h /usr/local/include/
	sudo ldconfig

# Uninstall
uninstall:
	sudo rm -f /usr/local/lib/$(CONTROLLER_LIB)
	sudo rm -f /usr/local/include/ERobMotorController.h
	sudo ldconfig

# Help
help:
	@echo "Available targets:"
	@echo "  all       - Build library and example (default)"
	@echo "  clean     - Clean build artifacts"
	@echo "  install   - Install library to system (requires sudo)"
	@echo "  uninstall - Remove library from system (requires sudo)"
	@echo "  help      - Show this help"
	@echo ""
	@echo "Prerequisites:"
	@echo "  1. Build SOEM library first: cd build && make"
	@echo "  2. Ensure libsoem.so exists in ./build/"
	@echo ""
	@echo "Usage:"
	@echo "  make                          # Build everything"
	@echo "  sudo ./motor_controller_example enp3s0  # Run example"

.PHONY: all clean install uninstall help