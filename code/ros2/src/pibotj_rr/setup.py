from setuptools import setup

package_name = 'pibotj_rr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            # nodo para mandar velocidades a los motores
            'input_node = pibotj_rr.input_node:main',
            # nodo que obtiene los valores de latitud y longitud del puerto serie
            'gps_node = pibotj_rr.gps_node:main',

            # nodo que publica la cámara correctamente
            'publisher_node = pibotj_rr.cameraPublisher:main',

            # nodo que publica cámara + tflite            
            'camera_test_node = pibotj_rr.camera_test_node:main',

            # nodo que publica cámara + tflite + Coral           
            'cameratflite_test_node = pibotj_rr.cameratflite_test_node:main'
            
        ],
    },
)

