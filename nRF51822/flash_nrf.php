#!/usr/bin/php
<?php
//
// first install php: 
// sudo apt-get install php
// and use i.e. chmod 700 flash_nrf.php
//

function sendcmd($tfp,$cmd) {
	$pos=0;
	fputs($tfp,$cmd."\r");
	do {
	    $result=fread($tfp,4096); 
	    // echo "(".$pos.") [".$result,"]\n";
	    $pos++;
	} while (stripos($result,">")===false && $pos<20);
        usleep(500);  // communication bonus
	return $result;
}

// main

if ($argc!=2) {
	echo "Usage: ".$argv[0]." file.hex\n";
	exit;
}

$ip = 'localhost';
$port = '4444';
$result = '';
$telnetfp = fsockopen($ip, $port); 

if ($telnetfp) {
	stream_set_timeout($telnetfp,20);  // 20s should be enough
	//echo fgets($telnetfp); 
	$result=fread($telnetfp,1024); 
	if (stripos($result,"Open On-Chip Debugger")!==false) {
		echo "We are in!\n";
		$result=sendcmd($telnetfp,"reset halt");
		echo "reset halt ok!\n";
		$result=sendcmd($telnetfp, "program ".$argv[1]," verify");
		echo "program maybe ok!\n";
		$result=sendcmd($telnetfp,"reset");
		echo "reset ok!\n";
		$result=sendcmd($telnetfp,"exit");
		echo "exit ok!\n";
	} else {
		echo "Sorry no openocd on Port 4444!\n";
	}
	fclose($telnetfp);
} else {
	echo "Sorry no openocd on Port 4444!\n";
}
?>
