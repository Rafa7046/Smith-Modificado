CC = gcc
CFLAGS = -shared -fPIC
TARGET = src\c_code\control_system.so
SRC = src\c_code\control_system.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET)
