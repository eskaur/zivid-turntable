echo "**** pylint ****"
pylint ./zivid_turntable || exit $?
echo "**** black ****"
black ./zivid_turntable --check --diff || exit $?
echo "**** mypy ****"
mypy ./zivid_turntable || exit $?
echo "**** SUCCESS ****"