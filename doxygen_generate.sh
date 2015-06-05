#!/bin/bash

# Script file to generate doxygen output for the last_letter project packages
# Exports XML output to construct visual representations of the class contents and relations
# See here for more info: http://stackoverflow.com/questions/9484879/graphviz-doxygen-to-generate-uml-class-diagrams/9488742#9488742
# Also invokes the doxygraph project to create an alternative, interactive visual output
# See here for installation info: https://code.google.com/p/doxygraph/source/browse/README

# Generate Doxygen output
LAST_LETTER_DIR=`rospack find last_letter`
cd $LAST_LETTER_DIR
doxygen doc/doxygen/Doxyfile

# Generate Doxygraph output, requires Doxygen XML output

DOXYGRAPH_ROOT_DIR=`echo /home/george/Software/doxygraph`
cd $DOXYGRAPH_ROOT_DIR
perl doxygraph/doxygraph $LAST_LETTER_DIR/doc/doxygen/xml/index.xml doxyviz/htdocs/graph.dot
cd doxyviz/htdocs
cp * $LAST_LETTER_DIR/doc/doxygraph