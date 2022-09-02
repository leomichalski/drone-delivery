#### Explicação dos arquivos

* [main.py](./main.py): arquivo que roda o programa e faz tudo funcionar em conjunto.

* [videosource.py](./videosource.py): pega as imagens da câmera.
* [videowebstreaming.py](./videowebstreaming.py): transmite as imagens da câmera.
* [arucodetector.py](./arucodetector.py): arquivo que encapsula o detector de marcadores aruco.
* [topics.py](./topics.py): tópicos da arquitetura "Publish/Subscribe".
* [msgs.py](./msgs.py): mensagens da arquitetura "Publish/Subscribe".
* [utils.py](./utils.py): métodos aleatórios que ajudam em algo (tipo um "miscellaneous").
* [rosbridge.py](./rosbridge.py): ponte para fazer o código em Python na arquitetura "Publish/Subscribe" funcionar com o código em ROS (que está em outra arquitetura "Publish/Subscribe").
* [requirements.txt](./requirements.txt): arquivo com as versões das bibliotecas Python utilizadas.


#### Como rodar

```
python3 main.py
```


#### Como instalar

```
pip3 install requirements.txt
```


