:: 如有个批处理a.bat在D:\test文件夹下 
:: a.bat内容为 cd /d %~dp0
:: 在这里
:: cd /d 表示直接转换到后面的路径，否则如果切换盘符，就需要再输入盘符才能切换路径
:: cd /d %~dp0的意思就是cd /d d:\test
:: %0代表批处理本身 d:\test\a.bat 
:: ~dp是变量扩充
:: d既是扩充到分区号 d: 
:: p就是扩充到路径 \test 
:: dp就是扩充到分区号路径 d:\test
::
:: 切换目录到Project\MDK-ARM
cd /d "%~dp0Project\MDK-ARM"
start Project.uvprojx