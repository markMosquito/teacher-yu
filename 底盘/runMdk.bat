:: ���и�������a.bat��D:\test�ļ����� 
:: a.bat����Ϊ cd /d %~dp0
:: ������
:: cd /d ��ʾֱ��ת���������·������������л��̷�������Ҫ�������̷������л�·��
:: cd /d %~dp0����˼����cd /d d:\test
:: %0������������ d:\test\a.bat 
:: ~dp�Ǳ�������
:: d�������䵽������ d: 
:: p�������䵽·�� \test 
:: dp�������䵽������·�� d:\test
::
:: �л�Ŀ¼��Project\MDK-ARM
cd /d "%~dp0Project\MDK-ARM"
start Project.uvprojx