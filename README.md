
#### Explicação dos arquivos
* **imav2022**: pasta do repositório.
  * **flight**: pasta do voo e das simulações de voo.
  * **vision**: pasta do código relacionado a Visão Computacional.
  * **scripts**: pasta com scripts úteis. Alguns deles são rodados quando a raspberry é ligada.

#### Como rodar o código na Raspberry 4 (ou 3B+) conectada na PX4
```
# TODO: um fps para o aruco e outro para o streaming (video saver n importa)
taskset -c 0,1,2 python3 main.py --webstream-video --detect-aruco --using-ros  --frames-per-second 7
# usar a flag "--using-vpn" se for usar a vpn para o video streaming

taskset -c 3 roslaunch mavros apm.launch
taskset -c 3 rosrun sim00 mission
# taskset -c 3 rosrun sim01 mission00
# taskset -c 3 rosrun sim01 mission01
```
