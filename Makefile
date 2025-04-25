
# Use the pokytoolchain compiler
#	First, do source /opt/fsl-imx-xwayland/5.15-kirkstone/environment-setup-armv8a-poky-linux
CC ?= gcc
LDFLAGS = -lgpiod 

SRC_DIR = src
TEST_DIR = test
OBJ_DIR = bin

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/*.c)
TEST_FILES = $(TEST_DIR)/main.c
OBJ_FILES = $(SRC_FILES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
TEST_OBJ = $(TEST_FILES:$(TEST_DIR)/%.c=$(OBJ_DIR)/%.o)

# Final executable
TARGET = mfrc522_module

all: $(TARGET)

$(TARGET): $(OBJ_FILES) $(TEST_OBJ)
	$(CC) $^ -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: $(TEST_DIR)/%.c
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(TARGET)