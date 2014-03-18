<?php
$inFile = $_SERVER['argv'][1];
$outFile = $_SERVER['argv'][2];
$f = fopen($inFile, "r");
$spm_pagesize = 256;

$out = "";
while(!feof($f)) {
	$payload = fread($f, $spm_pagesize);
	$out .= sprintf("%04X", $spm_pagesize);
	for($i=0; $i<$spm_pagesize; $i++) {
		if ($i >= strlen($payload)) {
			$out .= "00";
		} else {
			$out .= sprintf("%02X", ord($payload[$i]));
		}
	}
}
file_put_contents($outFile, $out);
