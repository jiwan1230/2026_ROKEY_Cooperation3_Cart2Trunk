# Cart2Trunk web/backend

## 테스트 실행 방법

이 디렉터리에서 pytest를 실행할 때는 항상 아래 명령을 그대로 사용한다.

```bash
cd isaacpjt/Cart2Trunk/web/backend
source venv/bin/activate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -v
```

### 왜 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`이 항상 필요한가

이 컴퓨터는 ROS2 humble이 전역으로 source되어 있어 `PYTHONPATH`에
`/opt/ros/humble/lib/python3.10/site-packages`가 포함된다. 그 경로 안의
`launch_testing`, `launch_testing_ros_pytest_entrypoint` 두 패키지가 pytest의
setuptools entry-point 메커니즘을 통해 자동으로 플러그인으로 로드되려다
크래시한다(각각 `ModuleNotFoundError: No module named 'yaml'`,
`pluggy._manager.PluginValidationError: unknown hook
'pytest_launch_collect_makemodule'`). `-p no:X` 방식의 개별 차단으로는
막히지 않으므로, entry-point 플러그인 자동탐색 자체를 끄는
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`을 실행 전에 반드시 설정해야 한다.
`pytest.ini`의 `testpaths`/`python_files` 설정만으로는 이 문제가 해결되지
않는다(자세한 내용은 `pytest.ini`의 주석 참고).

`algorism/` 쪽 테스트도 전역 pytest 플러그인 충돌 때문에 항상 `-p no:anyio`가
필요한 것과 같은 종류의 문제다.
