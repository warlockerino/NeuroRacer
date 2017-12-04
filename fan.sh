#!/bin/bash

fan(){
if(($1 >= 0)) && (($1<=255));then
echo $1
else
echo "fanspeed must be 0-255" 
fi
}

