<?php
$valor_estado=$_POST['valor_estado'];

switch ($valor_estado) {
	case 1:
		exec('sudo python3 /var/www/html/control/py/ahead.py');
		break;
	case 2:
		exec('sudo python3 /var/www/html/control/py/left.py');
		break;
	case 3:
		exec('sudo python3 /var/www/html/control/py/right.py');
		break;
	case 4:
		exec('sudo python3 /var/www/html/control/py/back.py');
		break;
	default:
		exec('sudo python3 /var/www/html/control/py/stop.py');
		break;
}
?>
