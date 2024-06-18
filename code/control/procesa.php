<?php
$valor_estado=$_POST['valor_estado'];

file_put_contents('/var/www/html/control/log/debug.log', "Estado recibido: " . $valor_estado . PHP_EOL, FILE_APPEND);
switch ($valor_estado) {
	case 1:
		exec('sudo python /var/www/html/control/py/ahead.py');
		break;
	case 2:
		exec('sudo python /var/www/html/control/py/left.py');
		break;
	case 3:
		exec('sudo python /var/www/html/control/py/right.py');
		break;
	case 4:
		exec('sudo python /var/www/html/control/py/back.py');
		break;
	case 5:
		exec('sudo python /var/www/html/control/py/stop.py');
		break;
	default:
		exec('sudo python /var/www/html/control/py/stop.py');
		break;
}

// Log the command to be executed
file_put_contents('/var/www/html/control/log/debug.log', "Comando ejecutado: " . $command . PHP_EOL, FILE_APPEND);

// Execute the command and log the output
$output = shell_exec($command . ' 2>&1');
file_put_contents('/var/www/html/control/log/debug.log', "Salida: " . $output . PHP_EOL, FILE_APPEND);

echo "Comando ejecutado: " . $command;
?>
