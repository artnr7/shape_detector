NPROC := $(shell expr $$(nproc) / 2)
ifeq ($(shell expr $(NPROC) \< 1), 1)
	NPROC := 1 
endif

.PHONY: all install f clang_f cmake_f output


default: r

DATASETS := ../dataset

install: 
	@cmake --build build --target shape_detector --parallel $(NPROC)

# run:
# 	@cmake --build build --target shape_detector --parallel $(NPROC)
# 	@cmake --build build --target run_shape_detector --parallel $(NPROC)

run:
# 	@./install_d/shape_detector $(DATASETS)/pedestrian.jpg
	@./install_d/shape_detector $(DATASETS)/stop.jpg

r: output f install run

output:
	@mkdir -p output

del:
	@rm -rf build .cache output

f: clang_f cmake_f
	@echo 'Format done'

clang_f:
	@clang-format -i -style=Google shape_detector.cc

cmake_f:
	@cmake-format -i CMakeLists.txt
