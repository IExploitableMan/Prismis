CC       := cc
CFLAGS   := -Iinclude -O3 -ffast-math -march=native -D_REENTRANT
LDFLAGS  := -lm -lSDL2 -lSDL2_image

C3C      := c3c
C3FLAGS := -l SDL2 -O5 --safe=no --optlevel=max --unroll-loops=yes --slp-vectorize=yes --loop-vectorize=yes --merge-functions=yes --single-module=yes

TARGET    := prismis
SRC       := prismis.c

C3_SRC    := prismis.c3 include/sdl2.c3i
C3_OUT    := prismis_c3

BUILD_DIR := build
OBJ       := $(BUILD_DIR)/prismis.o

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR):
	mkdir -p $@

$(OBJ): $(SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS)

run: $(BUILD_DIR)/$(TARGET)
	$(CC) $(CFLAGS) -o $(BUILD_DIR)/$(TARGET) $(SRC) $(LDFLAGS)
	$(BUILD_DIR)/$(TARGET)

c3: | $(BUILD_DIR)
	$(C3C) compile $(C3_SRC) -o $(BUILD_DIR)/$(C3_OUT) $(C3FLAGS)

c3-run:
	$(C3C) compile-run $(C3_SRC) $(C3FLAGS) -o $(BUILD_DIR)/$(C3_OUT)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all c3 c3-run clean
