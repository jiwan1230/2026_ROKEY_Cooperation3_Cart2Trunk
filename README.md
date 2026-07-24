# Cart2Trunk 웹 플래너 - 실행 방법

두 개의 터미널에서 백엔드와 프론트엔드를 각각 띄워야 한다.

## 1. 백엔드 (Flask, API 전용 - 화면 없음)

```bash
cd isaacpjt/Cart2Trunk/web/backend
source venv/bin/activate   # 최초 1회: python3 -m venv venv && pip install -r requirements.txt
python app.py
```

`http://localhost:5000/api/health`가 `{"status":"ok"}`를 반환하면 정상.

## 2. 프론트엔드 (React+Vite - 실제로 보고 쓰는 화면)

```bash
cd isaacpjt/Cart2Trunk/web/frontend
npm install   # 최초 1회
npm run dev
```

브라우저로 `http://localhost:5173`을 열면 실제 UI가 뜬다. 백엔드가 5000번 포트에서
먼저(또는 동시에) 켜져 있어야 트렁크 스캔 파일/박스 프리셋 목록을 불러올 수 있다.

## 테스트

```bash
# 백엔드
cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python -m pytest -v

# 프론트엔드
cd isaacpjt/Cart2Trunk/web/frontend && npm test
```
