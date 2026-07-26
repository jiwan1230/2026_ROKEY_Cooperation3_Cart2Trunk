"""
app.py
Cart2Trunk 웹 플래너 백엔드 진입점. algorism/ 계산 로직은 algorism_bridge.py를
통해서만 호출한다 - 이 파일은 라우트 등록과 에러 처리만 담당한다.
"""
from flask import Flask, jsonify
from flask_cors import CORS
from werkzeug.exceptions import HTTPException


def create_app():
    app = Flask(__name__)
    CORS(app, origins=["http://localhost:5173"])

    from routes.resources import resources_bp
    app.register_blueprint(resources_bp)

    from routes.plan import plan_bp, ApiError
    app.register_blueprint(plan_bp)

    from routes.approval import approval_bp
    app.register_blueprint(approval_bp)

    from routes.vision import vision_bp
    app.register_blueprint(vision_bp)

    from routes.robot import robot_bp
    app.register_blueprint(robot_bp)

    @app.errorhandler(ApiError)
    def handle_api_error(err):
        return err.to_response()

    @app.get("/api/health")
    def health():
        return jsonify({"status": "ok"})

    @app.errorhandler(Exception)
    def handle_unexpected_error(err):
        if isinstance(err, HTTPException):
            return err
        return jsonify({
            "error_code": type(err).__name__,
            "cause": str(err),
            "action": "입력값(트렁크 스캔 파일, 박스 목록, 마진/우선순위 파라미터)을 확인한 뒤 다시 시도하세요.",
        }), 500

    return app


app = create_app()

if __name__ == "__main__":
    # threaded=True: 로봇 트리거 더미 지연(1.5초) 동안 다른 요청(예: 트렁크
    # 맵 3초 폴링)이 밀리지 않게 한다 - 순수 time.sleep 대기라 스레드 경합
    # 위험은 없다.
    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True)
