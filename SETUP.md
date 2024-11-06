
# Rokey ROS 프로젝트

이 프로젝트는 ROS2 기반의 `Rokey ROS` 시스템을 개발하고 관리하기 위한 저장소입니다. 아래는 프로젝트 구축, 업데이트, 개발 및 실행 방법에 대한 가이드를 제공합니다.

---

## 📂 프로젝트 클론 및 환경 설정

1. **ROS 워크스페이스로 이동**
   ```bash
   cd ~/your_workspace/src
   ```

2. **프로젝트 클론**
   ```bash
   git clone https://github.com/sepengsu/rokey_ros.git
   ```

3. **워크스페이스 빌드**
   ```bash
   cd ~/your_workspace
   colcon build
   ```

4. **환경 설정**
   ```bash
   source install/setup.bash
   ```

---

## 🔄 프로젝트 업데이트

GitHub의 최신 변경 사항을 로컬에 반영하려면 다음 명령어를 사용하세요:

```bash
cd ~/your_workspace/src/rokey_ros
git pull
cd ~/your_workspace
colcon build
source install/setup.bash
```

---

## 🚀 실행 방법

1. ROS 노드 실행:
   ```bash
   ros2 run <패키지_이름> <노드_이름>
   ```

2. 데이터베이스 초기화 (필요시):
   데이터베이스 관련 초기화 코드 또는 스크립트를 실행해야 합니다.

---

## 🛠️ 개발 및 커스터마이징

### 1. 새 브랜치 생성
새로운 기능을 개발하려면 새 브랜치를 생성합니다:
```bash
cd ~/your_workspace/src/rokey_ros
git checkout -b feature/your-feature-name
```

### 2. 변경 사항 커밋
변경 사항을 저장합니다:
```bash
git add .
git commit -m "Add new feature"
```

### 3. 원격 저장소로 푸시
GitHub에 변경 사항을 반영합니다:
```bash
git push origin feature/your-feature-name
```

---

## 🤝 협업 및 충돌 해결

1. 최신 `main` 브랜치 가져오기:
   ```bash
   cd ~/your_workspace/src/rokey_ros
   git checkout main
   git pull
   ```

2. 브랜치 병합:
   ```bash
   git merge main
   ```

3. 충돌 해결 후 커밋:
   충돌이 발생하면 파일을 수정하고 저장합니다:
   ```bash
   git add .
   git commit -m "Resolve merge conflict"
   ```

---

## 📜 규칙 및 워크플로우

- 모든 변경 사항은 **새 브랜치**에서 작업합니다.
- 작업이 완료되면 **Pull Request**를 생성하여 코드 리뷰를 진행합니다.
- `main` 브랜치는 항상 최신 상태로 유지합니다.

---

## 📚 참고 자료

- ROS2 공식 문서: [https://docs.ros.org/](https://docs.ros.org/)
- Git 사용법: [https://git-scm.com/](https://git-scm.com/)

---

이 문서를 활용하여 프로젝트를 원활히 관리하고 개발하세요! 😊
