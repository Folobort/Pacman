CC=g++


all: graph.cpp Parser.cpp Point.cpp Kuplet.cpp PointFactory.cpp Signature.cpp
	$(CC) graph.cpp Parser.cpp Point.cpp Kuplet.cpp PointFactory.cpp Signature.cpp

clean:
	rm -rf *.o *.dot *.png
