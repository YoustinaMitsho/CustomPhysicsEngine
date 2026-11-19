#include <iostream>
#include <SFML/Graphics.hpp>
#include <math.h>
#include <cmath>

using namespace sf;
using namespace std;

const float M_PI = 3.14159265358979323846;
constexpr float PPM = 100.0f;

struct Vec2 {
	float x, y;

	// Construction
	Vec2(float x = 0, float y = 0) : x(x), y(y) {}

	// Arithmetic operators
	Vec2 operator+(const Vec2& o) const { return { x + o.x, y + o.y }; }
	Vec2 operator-(const Vec2& o) const { return { x - o.x, y - o.y }; }
	Vec2 operator*(float s)       const { return { x * s, y * s }; }
	Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
	Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }

	// Length and normalization
	float LengthSq() const { return x * x + y * y; }
	float Length()   const { return std::sqrt(LengthSq()); }
	Vec2  Normalized() const {
		float len = Length();
		return len > 0.0001f ? Vec2{ x / len, y / len } : Vec2{};
	}

	// Dot product: >0 same dir, =0 perp, <0 opposite
	float Dot(const Vec2& o) const { return x * o.x + y * o.y; }
	float Cross(const Vec2& o) const { return x * o.y - y * o.x; }
};

struct RigidBody {
	Vec2 prevPosition;
	Vec2 position;
	Vec2 velocity;
	Vec2 acceleration;
	Vec2 forceAccum;
	float mass, invMass;
	float restitution = 0.6f;
	float linearDamping = 0.99f;

	// Angular
	float angle = 0.f;
	float angularVelocity = 0.f;
	float torqueAccum = 0.f;
	float lastTorque = 0.f;
	float inertia, invInertia;

	void AddForce(Vec2 f) { forceAccum += f; }
	void AddForceAtPoint(Vec2 f, Vec2 point) {
		forceAccum += f;
		Vec2 r = point - position;
		torqueAccum += r.Cross(f);
	}
	void ApplyDownwardForceAtPoint(Vec2 worldClickPoint, float downwardMagnitude) {
		Vec2 r = worldClickPoint - position;

		if (std::abs(r.x) < 3.0f) {
			r.x = 0.0f;
		}

		Vec2 F = { 0.0f, downwardMagnitude };
		float torque = r.Cross(F);
		AddTorque(torque);
	}
	void SetInertia(float I) {
		inertia = I;
		invInertia = (I != 0.0f) ? 1.0f / I : 0.0f;
	}

	void AddTorque(float t) {
		torqueAccum += t;
		lastTorque = t;
	}
};

struct AABB {
	Vec2 min, max;
	AABB(Vec2 min, Vec2 max) {
		this->min = min;
		this->max = max;
	}
};

constexpr float FIXED_DT = 1.0f / 60.0f;
constexpr float GROUNDY = 560.0f;
constexpr float RADIUS = 15.f;

vector<RigidBody> bodies;

void ApplyForces(RigidBody& b, float dt) {
	if (b.invMass == 0.f) return;

	b.AddForce({ 0.f, b.mass * 9.81f });

	const float k = 0.02f;
	b.AddForce(b.velocity * -k);

	Vec2 accel = b.forceAccum * b.invMass;
	b.velocity += accel * dt;
	b.position += b.velocity * dt;

	b.velocity = b.velocity * b.linearDamping;

	b.forceAccum = {};
}

float BounceVelocity(float impactVel, float e) {
	return -impactVel * e;
}

void ApplyImpulse(RigidBody& a, RigidBody& b, Vec2 impulse) {
	a.velocity += impulse * a.invMass;
	b.velocity -= impulse * b.invMass;
}

void IntegrateRigidBody(RigidBody& b, float dt) {
	if (b.invMass == 0.f) return;

	Vec2 accel = b.forceAccum * b.invMass;
	b.velocity += accel * dt;
	b.position += b.velocity * dt;

	float angAccel = b.torqueAccum * b.invInertia;
	b.angularVelocity += angAccel * dt;
	b.angle += b.angularVelocity * dt;

	b.forceAccum = {};
	b.torqueAccum = 0.f;
}

float RectInertia(float m, float w, float h) {
	return m * (w * w + h * h) / 12.0f;
}

void PhysicsStep(float dt) {
	for (auto& body : bodies) {
		body.prevPosition = body.position;
		ApplyForces(body, dt);

		if (body.position.y + RADIUS > GROUNDY && body.velocity.y > 0) {
			body.position.y = GROUNDY - RADIUS;
			body.velocity.y = BounceVelocity(body.velocity.y, body.restitution);
		}
	}
}

void Render(RenderWindow& window, vector<RectangleShape>& shapes, float alpha) {
	window.clear(Color(30, 30, 30));
	for (size_t i = 0; i < bodies.size(); i++)
	{
		float renderX = bodies[i].prevPosition.x + (bodies[i].position.x - bodies[i].prevPosition.x) * alpha;
		float renderY = bodies[i].prevPosition.y + (bodies[i].position.y - bodies[i].prevPosition.y) * alpha;
		shapes[i].setPosition(Vector2f{ renderX, renderY });
		window.draw(shapes[i]);
	}
	window.display();
}

bool checkCollision(Vector2f c1, float r1, Vector2f c2, float r2) {
	float dx = c2.x - c1.x;
	float dy = c2.y - c1.y;
	float distSq = dx * dx + dy * dy;
	float radiusSum = r1 + r2;
	return distSq < radiusSum * radiusSum;
}

bool CircleCircle(Vec2 p1, float r1, Vec2 p2, float r2) {
	float dist2 = (p2 - p1).LengthSq();
	float rSum = r1 + r2;
	return dist2 <= rSum * rSum;
}

bool AABBvsAABB(const AABB& a, const AABB& b) {
	if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
	if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
	return true;
}

bool CircleAABB(Vec2 c, float r, const AABB& b) {
	Vec2 closest = {
		clamp(c.x, b.min.x, b.max.x),
		clamp(c.y, b.min.y, b.max.y)
	};
	return (closest - c).LengthSq() <= r * r;
}
/*-------------------------------------------------------------------------------------------------------------------*/
constexpr float GRAVITY = 500.0f;
constexpr float DRAG_COEFF = 1.8f;

struct Confetti : RigidBody {
	Color color;
	float lifetime = 0.0f;
	float maxLifetime = 1.5f;
};

struct Rocket : RigidBody {
	float rocketSpeed = 350.0f;
	bool exploded = false;
	CircleShape body;
	RectangleShape HitBox;
	vector<Vec2> trail;
};

//Vec2 Task1_RocketInitialVelocity(const Rocket& rocket, Vec2 mousePosition) {
//	Vec2 direction = (mousePosition - rocket.position).Normalized();
//	return direction * rocket.rocketSpeed;
//}

Vec2 Task1_RocketInitialVelocity(const Rocket& rocket, Vec2 mousePosition) {
	float dy = rocket.position.y - mousePosition.y;
	if (dy < 20.0f) dy = 20.0f;

	float vy = -std::sqrt(2.0f * GRAVITY * dy);
	float timeToApex = -vy / GRAVITY;
	float dx = mousePosition.x - rocket.position.x;
	float vx = dx / timeToApex;

	return Vec2{ vx, vy } *(rocket.rocketSpeed / 350.0f);
}

void Task1_SpawnRocket(vector<Rocket>& rockets, Vec2 windowSize, Vec2 mousePosition, const float radius, const float speed) {
	Rocket r;
	r.mass = 1.0f;
	r.invMass = 1.0f;
	r.rocketSpeed = speed;
	r.position = { windowSize.x * 0.5f, windowSize.y };
	r.prevPosition = r.position;
	r.velocity = Task1_RocketInitialVelocity(r, mousePosition);

	r.body.setRadius(radius);
	r.body.setOrigin({ radius, radius });
	r.body.setFillColor(Color::White);
	r.body.setPosition({ r.position.x, r.position.y });

	r.HitBox.setSize({ radius * 4 , radius * 4 });
	r.HitBox.setOrigin({ radius * 2, radius * 2 });
	r.HitBox.setFillColor(Color::Transparent);
	r.HitBox.setOutlineColor(Color::Red);
	r.HitBox.setOutlineThickness(1.0f);
	r.HitBox.setPosition({ r.position.x, r.position.y });

	rockets.push_back(r);
}

void Task1_RocketUpdate(vector<Rocket>& rockets, float dt) {
	for (auto& r : rockets) {
		r.prevPosition = r.position;
		IntegrateRigidBody(r, dt);

		r.body.setPosition({ r.position.x, r.position.y });
		r.HitBox.setPosition({ r.position.x, r.position.y });

		r.trail.push_back(r.position);
		if (r.trail.size() > 15) {
			r.trail.erase(r.trail.begin());
		}
	}
}

void TaskExtraDrawTail(RenderWindow& window, const Rocket& r, const float rocketRadius) {
	if (r.trail.empty())
	{
		return;
	}

	CircleShape trailShape(rocketRadius);

	const int count = static_cast<int>(r.trail.size());

	for (int i = 0; i < count; i++)
	{
		float ratio = static_cast<float>(i + 1) / static_cast<float>(count);
		float radius = rocketRadius * ratio;

		trailShape.setRadius(radius);
		trailShape.setOrigin({ radius, radius });
		trailShape.setPosition({
			r.trail[i].x,
			r.trail[i].y
			});

		Color color = Color::White;
		color.a = static_cast<uint8_t>(180.0f * ratio);
		trailShape.setFillColor(color);

		window.draw(trailShape);
	}
}

void Task2_ApplyRocketGravity(vector<Rocket>& rockets) {
	for (auto& r : rockets) {
		r.AddForce({ 0.0f, r.mass * GRAVITY });
	}
}


float RandomFloat(float minVal, float maxVal) {
	float t = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	return minVal + t * (maxVal - minVal);
}

Color RandomFireworkColor() {
	uint8_t r = static_cast<uint8_t>(rand() % 156 + 100);
	uint8_t g = static_cast<uint8_t>(rand() % 156 + 100);
	uint8_t b = static_cast<uint8_t>(rand() % 156 + 100);
	return Color(r, g, b);
}

void Task4_SpawnConfetti(vector<Confetti>& confetti, Vec2 explosionPos) {
	constexpr int SPARK_COUNT = 80;

	for (int i = 0; i < SPARK_COUNT; i++) {
		Confetti c;
		c.position = explosionPos;
		c.prevPosition = explosionPos;

		float angle = RandomFloat(0.0f, 2.0f * M_PI);
		float speed = RandomFloat(120.0f, 380.0f);
		c.velocity = { std::cos(angle) * speed, std::sin(angle) * speed };

		c.mass = RandomFloat(0.5f, 2.0f);
		c.invMass = 1.0f / c.mass;

		c.color = RandomFireworkColor();
		c.lifetime = 0.0f;
		c.maxLifetime = RandomFloat(1.4f, 2.2f);

		confetti.push_back(c);
	}
}

void Task3_CheckRocketApex(vector<Rocket>& rockets, vector<Confetti>& confetti) {
	for (int i = (int)rockets.size() - 1; i >= 0; i--) {
		if (rockets[i].velocity.y >= 0.0f) {
			Task4_SpawnConfetti(confetti, rockets[i].position);
			rockets.erase(rockets.begin() + i);
		}
	}
}

void Task5_ApplyConfettiForces(vector<Confetti>& confetti, float windForceX) {
	for (auto& c : confetti) {
		c.AddForce({ 0.0f, c.mass * GRAVITY });
		c.AddForce(c.velocity * -DRAG_COEFF);
		c.AddForce({ windForceX, 0.0f });
	}
}

void Task5_UpdateConfetti(vector<Confetti>& confetti, float dt) {
	for (int i = (int)confetti.size() - 1; i >= 0; i--) {
		confetti[i].prevPosition = confetti[i].position;
		confetti[i].lifetime += dt;

		if (confetti[i].lifetime >= confetti[i].maxLifetime) {
			confetti.erase(confetti.begin() + i);
			continue;
		}
		IntegrateRigidBody(confetti[i], dt);
	}
}

void Task6_UpdateHUD(Text& hudText, float frameTime, size_t sparkCount, float windForceX, bool autoLaunch, int& score, float rocketRadius, float rocketSpeed) {
	static float fpsTimer = 0.0f;
	static int frameCounter = 0;
	static int currentFPS = 60;

	fpsTimer += frameTime;
	frameCounter++;
	if (fpsTimer >= 0.25f) {
		currentFPS = static_cast<int>(frameCounter / fpsTimer);
		fpsTimer = 0.0f;
		frameCounter = 0;
	}

	string hudString =
		"FPS: " + to_string(currentFPS) +
		"\t |\t Sparks: " + to_string(sparkCount) +
		"\t |\t Wind: " + to_string(static_cast<int>(windForceX)) +
		"\t |\t Score: " + to_string(static_cast<int>(score)) +
		"\t |\t Radius: " + to_string(static_cast<int>(rocketRadius)) +
		"\t |\t Speed: " + to_string(static_cast<int>(rocketSpeed));

	hudText.setString(hudString);
}

void Task7_HandleKeyInput(Keyboard::Scancode code, float& windForceX, bool& autoLaunch, vector<Rocket>& rockets, vector<Confetti>& confetti) {
	constexpr float WIND_STEP = 50.0f;

	if (code == Keyboard::Scancode::Left) {
		windForceX -= WIND_STEP;
	}
	else if (code == Keyboard::Scancode::Right) {
		windForceX += WIND_STEP;
	}
	else if (code == Keyboard::Scancode::Down) {
		windForceX = 0.0f;
	}
}

void Task8_UpdateAutoLaunch(bool autoLaunch, float& autoTimer, float dt, vector<Rocket>& rockets, Vec2 windowSize, const float radius, const float speed) {
	if (!autoLaunch) return;

	autoTimer += dt;
	constexpr float LAUNCH_INTERVAL = 0.6f;

	if (autoTimer >= LAUNCH_INTERVAL) {
		autoTimer = 0.0f;
		Vec2 randomTarget = {
			RandomFloat(120.0f, windowSize.x - 120.0f),
			RandomFloat(120.0f, windowSize.y * 0.55f)
		};
		Task1_SpawnRocket(rockets, windowSize, randomTarget, radius, speed);
	}
}

bool Task9Extra_HitRocketMidAir(Vector2f mousePosition, vector<Rocket>& rockets, vector<Confetti>& confetti, int& score) {
	for (int i = 0; i < rockets.size(); i++) {
		if (rockets[i].HitBox.getGlobalBounds().contains(mousePosition)) {
			if (Mouse::isButtonPressed(Mouse::Button::Left)) {
				Task4_SpawnConfetti(confetti, Vec2{ mousePosition.x, mousePosition.y });
				rockets.erase(rockets.begin() + i);
				score++;
				i--;
				return true;
			}
		}
	}
	return false;
}

void Task10Extra_ShakeScreen(RenderWindow& window, float& shakeTimer, float dt) {
	View view = window.getDefaultView();

	if (shakeTimer > 0.0f) {
		shakeTimer -= dt;
		constexpr float SHAKE_INTENSITY = 6.0f;
		float offsetX = RandomFloat(-SHAKE_INTENSITY, SHAKE_INTENSITY);
		float offsetY = RandomFloat(-SHAKE_INTENSITY, SHAKE_INTENSITY);
		view.move({ offsetX, offsetY });
	}

	window.setView(view);
}

void Task11Extra_IncreaseDefeculty(int score, float& rocketRadius, float& rocketSpeed) {
	static int lastMilestone = 0;
	static bool decreaseSizeTurn = true;

	int currentMilestone = score / 15;
	while (lastMilestone < currentMilestone) {
		lastMilestone++;

		bool canDecreaseSize = (rocketRadius > 5.0f);
		bool canIncreaseSpeed = (rocketSpeed < 700.0f);

		if (!canDecreaseSize && !canIncreaseSpeed) {
			break;
		}

		if (decreaseSizeTurn && canDecreaseSize) {
			rocketRadius -= 2.0f;
			if (rocketRadius < 5.0f) rocketRadius = 5.0f;
			decreaseSizeTurn = false;
		}
		else if (!decreaseSizeTurn && canIncreaseSpeed) {
			rocketSpeed += 50.0f;
			if (rocketSpeed > 700.0f) rocketSpeed = 700.0f;
			decreaseSizeTurn = true;
		}
		else if (canDecreaseSize) {
			rocketRadius -= 2.0f;
			if (rocketRadius < 5.0f) rocketRadius = 5.0f;
		}
		else if (canIncreaseSpeed) {
			rocketSpeed += 50.0f;
			if (rocketSpeed > 700.0f) rocketSpeed = 700.0f;
		}
	}
}

void Tas12Extra_MoveBarrel(Sprite& gun, const RenderWindow& window) {
	Vector2i mousePos = Mouse::getPosition(window);
	float windowWidth = static_cast<float>(window.getSize().x);
	float windowHeight = static_cast<float>(window.getSize().y);
	float gunWidth = static_cast<float>(gun.getTexture().getSize().x);
	float gunHeight = static_cast<float>(gun.getTexture().getSize().y);

	float centerX = (windowWidth - gunWidth) / 2.0f;
	float baseY = windowHeight - gunHeight;

	// Normalized mouse offset from screen center in [-1, 1]
	float normalizedOffset = (static_cast<float>(mousePos.x) - (windowWidth * 0.5f)) / (windowWidth * 0.5f);
	normalizedOffset = std::clamp(normalizedOffset, -1.0f, 1.0f);

	constexpr float MAX_BARREL_OFFSET = 400.0f;
	float offsetX = normalizedOffset * MAX_BARREL_OFFSET;

	gun.setPosition({ centerX + offsetX, baseY });
}

int main() {
	RenderWindow* window = new RenderWindow(VideoMode({ 900, 900 }), "Custom Physics Engine");
	window->setFramerateLimit(60);
	const Cursor crosshairCursor(Cursor::Type::Cross);
	window->setMouseCursor(crosshairCursor);

	Font font;
	if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) {
		return -1;
	}
	Text hudText(font);
	hudText.setCharacterSize(20);
	hudText.setFillColor(Color::White);
	hudText.setOutlineThickness(1);
	hudText.setOutlineColor(Color::Black);
	hudText.setPosition({ 20.0f, 20.0f });

	vector<Rocket> rockets;
	/*CircleShape rocketShape(25.0f);
	rocketShape.setOrigin({ 25.0f, 25.0f });
	rocketShape.setFillColor(Color::White);*/

	vector<Confetti> confetti;
	CircleShape sparkShape(5.f);
	sparkShape.setOrigin({ 5.f, 5.f });

	Texture bgText;
	bgText.loadFromFile("Bg.jpeg");

	Sprite bgSprite(bgText);
	Vector2u texSize = bgText.getSize();
	Vector2u winSize = window->getSize();
	bgSprite.setScale({ static_cast<float>(winSize.x) / texSize.x, static_cast<float>(winSize.y) / texSize.y });

	Texture gunTexture;
	gunTexture.loadFromFile("gun.png");
	Texture gunFireTexture;
	gunFireTexture.loadFromFile("gunfire.png");

	Sprite gun(gunTexture);
	gun.setPosition({
		(window->getSize().x - gunTexture.getSize().x) / 2.0f,
		static_cast<float>(window->getSize().y - gunTexture.getSize().y)
		});

	float accumulator = 0.0f;
	Clock clock;

	float windForceX = 0.0f;
	bool autoLaunch = true;
	float autoLaunchTimer = 0.0f;
	float rocketRadius = 10.0f;
	float rocketSpeed = 350.0f;
	int score = 0;
	bool isShoot = false;
	int shootDelay = 0;
	float shakeTimer = 0.0f;

	while (window->isOpen()) {
		float frameTime = clock.restart().asSeconds();
		while (const optional event = window->pollEvent()) {
			if (event->is<Event::Closed>()) {
				window->close();
			}
			else if (const auto* keyPressed = event->getIf<Event::KeyPressed>()) {
				if (keyPressed->scancode == Keyboard::Scancode::Escape) {
					window->close();
				}
				Task7_HandleKeyInput(keyPressed->scancode, windForceX, autoLaunch, rockets, confetti);
			}
			else if (const auto* mousePressed = event->getIf<Event::MouseButtonPressed>()) {
				if (mousePressed->button == Mouse::Button::Left) {
					Vector2f worldPos = window->mapPixelToCoords(mousePressed->position);
					//Task1_SpawnRocket(rockets, { 900.0f, 900.0f }, { worldPos.x, worldPos.y });
					if (Task9Extra_HitRocketMidAir(worldPos, rockets, confetti, score)) {
						shakeTimer = 0.2f;
						Task11Extra_IncreaseDefeculty(score, rocketRadius, rocketSpeed);
					}
					isShoot = true;
					shootDelay = 0;
				}
			}
		}
		//update
		if (frameTime > 0.25f) frameTime = 0.25f;
		accumulator += frameTime;


		while (accumulator >= FIXED_DT) {
			Task8_UpdateAutoLaunch(autoLaunch, autoLaunchTimer, FIXED_DT, rockets, { 900.0f, 900.0f }, rocketRadius, rocketSpeed);

			Task2_ApplyRocketGravity(rockets);
			Task1_RocketUpdate(rockets, FIXED_DT);
			//Task3_CheckRocketApex(rockets, confetti);

			for (auto& c : confetti) {
				IntegrateRigidBody(c, FIXED_DT);
			}
			Task5_ApplyConfettiForces(confetti, windForceX);
			Task5_UpdateConfetti(confetti, FIXED_DT);

			accumulator -= FIXED_DT;
		}

		Task6_UpdateHUD(hudText, frameTime, confetti.size(), windForceX, autoLaunch, score, rocketRadius, rocketSpeed);
		Task10Extra_ShakeScreen(*window, shakeTimer, frameTime);

		if (isShoot) {
			gun.setTexture(gunFireTexture);
			shootDelay++;
			if (shootDelay >= 10) {
				isShoot = false;
				shootDelay = 0;
			}
		}
		else {
			gun.setTexture(gunTexture);
		}

		Tas12Extra_MoveBarrel(gun, *window);

		window->clear(Color(30, 30, 30));
		window->draw(bgSprite);

		for (const auto& r : rockets) {
			TaskExtraDrawTail(*window, r, r.body.getRadius());
			//rocketShape.setPosition({ r.position.x, r.position.y });
			window->draw(r.HitBox);
			window->draw(r.body);
		}

		for (const auto& c : confetti) {
			float lifeRatio = 1.0f - (c.lifetime / c.maxLifetime);
			if (lifeRatio < 0.0f) lifeRatio = 0.0f;

			Color fadedColor = c.color;
			fadedColor.a = static_cast<uint8_t>(255.0f * lifeRatio);

			sparkShape.setPosition({ c.position.x, c.position.y });
			sparkShape.setFillColor(fadedColor);
			window->draw(sparkShape);
		}

		window->draw(gun);
		window->draw(hudText);

		window->display();
	}

	delete window;
}