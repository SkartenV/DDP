#!/bin/bash

NumeroSemilla=1
NumeroClientes=10
NumeroInstancia=1

while [ $NumeroSemilla -le 3 ]
do
    NumeroClientes=10
    while [ $NumeroClientes -le 45 ]
    do
        NumeroInstancia=1
        while [ $NumeroInstancia -le 5 ]
        do
            ./DDP Set_A1_Cust_"$NumeroClientes"_$NumeroInstancia $NumeroSemilla 30 30000 148 3 182 0.79
            let NumeroInstancia=$NumeroInstancia+1
        done
        let NumeroClientes=$NumeroClientes+5
    done
    let NumeroSemilla=$NumeroSemilla+1
done