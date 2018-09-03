python px_mkfw.py --prototype px4fmu-v4.prototype --image ..\mdk\obj\ob_trans.bin > ..\bin\ob_trans.px4


python px_uploader.py --port COM8 ..\bin\ob_trans.px4


pause


