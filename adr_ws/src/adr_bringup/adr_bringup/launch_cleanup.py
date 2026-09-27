"""launch 기동 시 이전 실행의 잔재 프로세스를 정리하는 공용 헬퍼.

sim.launch.py 와 step1.launch.py 가 같이 쓴다. 패턴 목록은 각자 다르지만(띄우는 노드가 다르다)
아래 주의사항은 공통이라 여기 한 번만 적는다.

왜 필요한가
  ros2 launch 가 비정상 종료하면(강제 kill, 크래시, 터미널 종료) 자식 노드가 고아로 살아남는다.
  다시 띄우면 같은 이름의 노드가 둘이 되는데, **latched(TRANSIENT_LOCAL) 토픽에서 특히 고약하다** —
  구독자가 옛 퍼블리셔의 낡은 샘플을 물어서, 고친 코드를 돌려도 옛 데이터가 보인다.
  실제로 /adr/planned_path 가 이 이유로 "떴다 안 떴다" 했다.

**기동 시에만** 쓸 것 — 종료 시엔 절대 쓰지 말 것
  예전에 sim.launch.py 가 OnShutdown 에서도 같은 pkill 을 했는데 연쇄 사고를 냈다:
    새 launch 기동 → cleanup 이 옛 launch 의 자식을 kill
    → 옛 launch 가 "자식이 죽었다"로 종료 절차 시작
    → 그 OnShutdown 이 같은 패턴으로 pkill → **방금 뜬 새 launch** 의 자식을 죽인다.
  pkill 패턴은 인스턴스를 구분하지 못하므로 종료 경로에서 쓰면 안 된다.
  고아가 남더라도 다음 기동의 cleanup 이 정리하므로 그것으로 충분하다.

패턴을 쓸 때
  - `pkill -f` 는 **cmdline 전체**와 매칭한다. 넓게 잡으면 무고한 프로세스(심지어 자기 셸)도 죽는다.
    가능하면 실행파일 경로나 `__node:=<이름>` 으로 좁힐 것.
  - launch 가 cwd 를 바꿔 띄우는 프로세스는 cmdline 이 상대경로일 수 있다(예: PX4 는 `./bin/px4`).
    절대경로 패턴만 두면 안 잡힌다.
"""
import subprocess


def kill_stale(patterns) -> list:
    """patterns = [(pkill -f 정규식, 설명)]. 실제로 죽인 것들의 설명 목록을 돌려준다."""
    killed = []
    for pattern, desc in patterns:
        r = subprocess.run(['pkill', '-9', '-f', pattern],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode == 0:        # 0 = 하나 이상 매칭해서 죽였음
            killed.append(desc)
    return killed
