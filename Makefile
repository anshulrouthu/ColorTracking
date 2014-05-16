
OPENCV_LIBS=    -lopencv_core                      \
                -lopencv_highgui                   \
                -lopencv_imgproc                   \
                -lopencv_ml                        \

TARGET:= color-track
INC:= 
CFLAGS:= -Wall -g -O2
C_SRC:=$(wildcard *.c)
CPP_SRC:=$(wildcard *.cpp)
OBJS:=$(C_SRC:.c=.o) $(CPP_SRC:.cpp=.o)

$(TARGET): $(OBJS)
	   @echo "Linking... $@"
	   @$(CXX) $(CFLAGS) $^ -o $@ $(OPENCV_LIBS)

%.o: %.cpp
	@echo "[CXX] $<"
	@$(CXX) $(CFLAGS) $(INC) -c $< -o $@

%.o: %.c
	@echo "[CC] $<"
	@$(CC) $(CFLAGS) $(INC) -c $< -o $@


.PHONY: clean
clean:
	 rm -f *.o $(TARGET)
