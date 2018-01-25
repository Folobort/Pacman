CC=g++

FILES = Bag.cpp ClassicGraph.cpp ClassicKuplet.cpp ClassicParser.cpp ClassicSignature.cpp graph.cpp GraphAux.cpp Kuplet.cpp Node.cpp Parser.cpp Point.cpp PointFactory.cpp Signature.cpp TreeDec.cpp TreeNode.cpp


all: $(FILES)
	$(CC) $(FILES)

clean:
	rm -rf *.o *.dot *.png a.out
