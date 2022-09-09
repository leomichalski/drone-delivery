#### Explicação dos arquivos

* [main.py](./main.py): arquivo que roda o programa e faz tudo funcionar em conjunto.

* [videosource.py](./videosource.py): pega as imagens da câmera.
* [videosaver.py](./videosaver.py): salva as imagens em formato de vídeo.
* [videowebstreaming.py](./videowebstreaming.py): transmite as imagens via HTTP.
* [arucodetector.py](./arucodetector.py): arquivo que encapsula o detector de marcadores aruco.
* [topics.py](./topics.py): tópicos da arquitetura "Publish/Subscribe".
* [msgs.py](./msgs.py): mensagens da arquitetura "Publish/Subscribe".
* [utils.py](./utils.py): métodos aleatórios que ajudam em algo.
* [rosbridge.py](./rosbridge.py): ponte para fazer o código em Python na arquitetura "Publish/Subscribe" funcionar com o código em ROS (que está em outra arquitetura "Publish/Subscribe").
* [requirements_raspberry.txt](./requirements_raspberry.txt): arquivo com as versões das bibliotecas Python utilizadas na raspberry.
* [requirements_gazebo.txt](./requirements_gazebo.txt): arquivo com as versões das bibliotecas Python utilizadas em conjunto com o gazebo.

#### Como instalar

```
# criar ambiente virtual
python3 -m venv --system-site-packages

# instalar na raspberry
pip3 install requirements_raspberry.txt

# instalar no computador com gazebo
pip3 install requirements_gazebo.txt
```
