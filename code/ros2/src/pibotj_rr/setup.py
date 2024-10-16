from setuptools import setup

package_name = 'pibotj_rr'
launch_folder_name = 'pibotj_rr/launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package for controlling a stepper motor and receiving input commands',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # nodo controlador para controlar los motores
            'motors_controller_node = pibotj_rr.motors_controller_node:main',
            
            # nodo que obtiene los valores de latitud y longitud del puerto serie
            'gps_node = pibotj_rr.gps_node:main',

            # nodo que publica la cámara correctamente
            'camera_node = pibotj_rr.camera:main',

            # nodo que publica cámara + tflite            
            'camera_tfv1_node = pibotj_rr.camera_tfv1_node:main',

            # nodo que publica cámara + tflite + Coral           
            'camera_tfv2_node = pibotj_rr.camera_tfv2_node:main',

            # nodo que publica cámara + tflite + Coral en funcionamiento y detecta bache en función de la máscara
            # publica un array con las coordenadas detectadas de la máscara   
            'camera_tfv3_node = pibotj_rr.camera_tfv3_node:main',

            # nodo que te dice en qué lado se sitúa el robot en función de la detección de las líneas
            'camera_dl_node = pibotj_rr.camera_dl_node:main',

            # nodo que ejecuta vff
            'camera_vff_node = pibotj_rr.camera_vff_node:main',

            # nodo que publica cámara + modelo pin hole (usando post-it)          
            'camera_pinhole_node = pibotj_rr.camera_pinhole_node:main',

            # nodo que publica cámara + modelo pin Hole + área funcionando
            'camera_pinholev2_node = pibotj_rr.camera_pinholev2_node:main',

            # nodo controlador para controlar los motores por la interfaz web
            'motors_controller_web_node = pibotj_rr.motors_controller_web_node:main',

            # nodo controlador para controlar los motores en el entorno controlado
            'motors_vff_node = pibotj_rr.motors_vff_node:main'

        ],
    },
)

