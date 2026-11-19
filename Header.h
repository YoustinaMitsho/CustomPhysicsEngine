#pragma once

#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <cmath>
#include <string>
#include <vector>

using namespace std;
using namespace sf;

// ==========================================
// GAME SETTINGS
// ==========================================

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

const float BULLET_SPEED = 450.0f;
const float GRAVITY = 500.0f;

// ==========================================
// GAME OBJECTS
// ==========================================

struct Cannon
{
    Vector2f position;
    float angle = 0.0f;

    CircleShape base;
    RectangleShape barrel;
};

struct Projectile
{
    CircleShape shape;
    Vector2f startPosition;
    Vector2f initialVelocity;
    float elapsedTime = 0.0f;
};

struct Target
{
    CircleShape shape;

    float homeY;
    float minY;
    float maxY;

    float speed;
    int direction = 1;
};

// ==========================================
// HELPER METHODS
// ==========================================

void CreateCannon(Cannon& cannon);
void CreateTargets(vector<Target>& targets);
bool SetupScoreText(Font& font, Text& scoreText);

void PollWindowEvents(RenderWindow& window, bool& shootRequested);

void UpdateTargets(vector<Target>& targets, float deltaTime);
void RemoveOffscreenBullets(vector<Projectile>& bullets);
void CheckCollisions(vector<Projectile>& bullets, vector<Target>& targets, int& score);

void DrawWorld(RenderWindow& window);
void DrawCannon(RenderWindow& window, const Cannon& cannon);
void DrawBullets(RenderWindow& window, const vector<Projectile>& bullets);
void DrawTargets(RenderWindow& window, const vector<Target>& targets);
void RenderFrame(RenderWindow& window, Text& scoreText, int score, const Cannon& cannon, const vector<Projectile>& bullets, const vector<Target>& targets);
void Task1RotateCannonWithMouse(Cannon& cannon, const RenderWindow& window);
void Task201FireBulletOnMouseClick(const Cannon& cannon, vector<Projectile>& bullets);
void Task202UpdateProjectileMotion(vector<Projectile>& bullets, float deltaTime);