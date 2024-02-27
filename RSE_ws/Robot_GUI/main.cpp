#include <QtWidgets>
#include <QDebug>
#include "TCPClient.h"

class Joypad : public QWidget {
    Q_OBJECT
public:
    QPushButton* navButton;
    QPushButton* sendButton; // Add Send button
    Joypad(QWidget *parent = nullptr) : QWidget(parent), pressed(false), navigationEnabled(false) {
        setMouseTracking(true);
        resetInnerCircle();
        setAutoFillBackground(true); // Enable auto-fill background
        QPalette palette = this->palette(); // Get the palette
        palette.setColor(QPalette::Window, parent->palette().color(QPalette::Window)); // Inherit background color from parent
        setPalette(palette); // Apply the palette
        client.connectToServer(ip, port);
        navButton = new QPushButton("Navigation", this);
        navButton->setFixedSize(100, 100); // Set fixed size to make it square
        connect(navButton, &QPushButton::clicked, this, &Joypad::toggleNavigation);
        updateNavButton(); // Initialize color

        // Create and connect the Send button
        sendButton = new QPushButton("Send", this);
        sendButton->setFixedSize(500, 50);
        connect(sendButton, &QPushButton::clicked, this, &Joypad::sendCoordinates);
    }

signals:
    void angleChanged(int angle); // Signal emitted when the angle changes

public slots:
    void setMaxLinearVelocity(int value) {
        maxLinearVel = static_cast<double>(value) / 100.0;
    }

    void setMaxAngularVelocity(int value) {
        maxAngularVel = static_cast<double>(value) / 100.0;
    }

    void toggleNavigation() {
        navigationEnabled = !navigationEnabled;
        updateNavButton();
    }

    void setGoalX(const QString& x) {
        bool ok;
        double xCoord = x.toDouble(&ok);
        if (ok) {
            goalX = xCoord;
            qDebug() << "Goal X coordinate set to:" << goalX;
        } else {
            qDebug() << "Invalid input for X coordinate";
        }
    }

    void setGoalY(const QString& y) {
        bool ok;
        double yCoord = y.toDouble(&ok);
        if (ok) {
            goalY = yCoord;
            qDebug() << "Goal Y coordinate set to:" << goalY;
        } else {
            qDebug() << "Invalid input for Y coordinate";
        }
    }

    void sendCoordinates() {
        qDebug() << "Sending coordinates - X:" << goalX << ", Y:" << goalY;
        client.sendData_Navigation(goalX, goalY, navigationEnabled);
    }

protected:
    TCPClient client;
    const char* ip = "127.0.0.1"; // IP address of the server
    int port = 12222345; // Port number of the server
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QRectF outerRect = rect().adjusted(5, 5, -5, -5);
        painter.setPen(QPen(Qt::black, 2));

        // Set the color for the area between outer and inner circles
        QColor innerAreaColor(230, 230, 230); // Adjust RGB values to make it brighter
        painter.setBrush(innerAreaColor);

        painter.drawEllipse(outerRect);

        if (pressed) {
            painter.setBrush(Qt::darkGray);
            painter.drawEllipse(innerRect);
        } else {
            painter.setBrush(Qt::lightGray);
            painter.drawEllipse(innerRect);
        }
    }

    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            pressed = true;
            mouseMoveEvent(event); // Update position immediately
            update();
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            pressed = false;
            resetInnerCircle(); // Reset the inner circle position
            update();
            client.sendData_Velocity(0.0, 0.0);
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        if (pressed) {
            QPointF center(width() / 2.0, height() / 2.0);
            QPointF pos = event->pos() - center;
            double distance = qSqrt(pos.x() * pos.x() + pos.y() * pos.y());
            if (distance > (width() / 4.0)) {
                // Normalize the position vector
                pos /= distance;
                pos *= (width() / 4.0); // Limit the distance to the outer circle radius
            }
            innerRect.moveCenter(center + pos); // Move the inner circle
            update();

            // Calculate the angle
            int angle = qRound(qRadiansToDegrees(qAtan2(-pos.y(), pos.x())));
            if (angle < 0) {
                angle += 360; // Ensure angle is in the range [0, 360)
            }
            if (angle != currentAngle) {
                currentAngle = angle;
                emit angleChanged(currentAngle); // Emit the angleChanged signal

                // Map angle to linear and angular velocities
                double linearDistance = qSqrt(pos.x() * pos.x() + pos.y() * pos.y());
                double maxDistance = width() / 4.0;
                double linearVel = maxLinearVel * (linearDistance / maxDistance); // Map linear velocity based on distance
                if (qFuzzyCompare(linearDistance, 0.0)) {
                    linearVel = 0.0; // Set linear velocity to 0 if the distance is 0
                } else {
                    // Determine the sign of the linear distance to determine direction
                    if (pos.y() > 0) {
                        linearVel *= -1; // If y-coordinate is negative, linear velocity is negative
                    }
                }
                double angularVel = maxAngularVel * qCos(qDegreesToRadians(angle));
                client.sendData_Velocity(linearVel, angularVel);
                // Debug the velocities
                qDebug() << "Linear Velocity:" << linearVel << ", Angular Velocity:" << angularVel;
            }
        }
    }

    void resizeEvent(QResizeEvent *event) override {
        int size = qMin(width(), height());
        innerRect = QRectF(width()/2 - size/8, height()/2 - size/8, size/4, size/4);
    }

private:
    QRectF innerRect;
    bool pressed;
    int currentAngle = 0;
    double maxLinearVel = 0.0; // Maximum linear velocity
    double maxAngularVel = 0.0; // Maximum angular velocity
    bool navigationEnabled;
    double goalX = 0.0; // Default goal X coordinate
    double goalY = 0.0; // Default goal Y coordinate
    void resetInnerCircle() {
        int size = qMin(width(), height());
        innerRect = QRectF(width()/2 - size/8, height()/2 - size/8, size/4, size/4);
        currentAngle = 0;
        emit angleChanged(currentAngle); // Emit the angleChanged signal for the center position
    }
    void updateNavButton() {
        if (navigationEnabled) {
            navButton->setStyleSheet("background-color: green");
        } else {
            navButton->setStyleSheet("background-color: red");
        }
    }
};

void createKnobsLayout(QWidget* parent, Joypad* joypad) {
    // Create and add velocity rotate knobs
    QHBoxLayout *knobsLayout = new QHBoxLayout;

    // Create linear velocity knob
    QDial *linearKnob = new QDial;
    linearKnob->setMinimum(0);
    linearKnob->setMaximum(100);
    linearKnob->setValue(0); // Initial value
    QObject::connect(linearKnob, &QDial::valueChanged, joypad, &Joypad::setMaxLinearVelocity);
    linearKnob->setNotchesVisible(true); // Show notches

    // Create angular velocity knob
    QDial *angularKnob = new QDial;
    angularKnob->setMinimum(0);
    angularKnob->setMaximum(100);
    angularKnob->setValue(0); // Initial value
    QObject::connect(angularKnob, &QDial::valueChanged, joypad, &Joypad::setMaxAngularVelocity);
    angularKnob->setNotchesVisible(true); // Show notches

    QVBoxLayout *linearLayout = new QVBoxLayout;
    QLabel *linearLabel = new QLabel("Linear Velocity");
    linearLabel->setAlignment(Qt::AlignCenter);
    linearLayout->addWidget(linearKnob);
    linearLayout->addWidget(linearLabel);
    linearLayout->setAlignment(Qt::AlignCenter); // Align linear layout to the center vertically

    QVBoxLayout *angularLayout = new QVBoxLayout;
    QLabel *angularLabel = new QLabel("Angular Velocity");
    angularLabel->setAlignment(Qt::AlignCenter);
    angularLayout->addWidget(angularKnob);
    angularLayout->addWidget(angularLabel);
    angularLayout->setAlignment(Qt::AlignCenter); // Align angular layout to the center vertically

    knobsLayout->addLayout(linearLayout);
    knobsLayout->addLayout(angularLayout);

    QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(parent->layout());
    if (mainLayout)
        mainLayout->addLayout(knobsLayout); // Add the knob layout to the main layout
}

void createJoypadLayout(QWidget* parent) {
    QVBoxLayout *layout = new QVBoxLayout(parent); // Create a vertical layout for the window

    // Add the Nav button first
    Joypad* joypad = new Joypad(parent); // Create the Joypad widget
    joypad->setMinimumSize(300, 300); // Set minimum size for the Joypad

    // Add a layout for the Nav button
    QHBoxLayout *navLayout = new QHBoxLayout;
    navLayout->addWidget(joypad->navButton);
    navLayout->addStretch(); // Add a stretch to push the button to the left
    layout->addLayout(navLayout); // Add the Nav button layout to the main layout

    // Add LineEdits for Goal Coordinates
    QLabel *xCoordLabel = new QLabel("X-Cordinates:");
    QLabel *yCoordLabel = new QLabel("Y-Cordinates:");


    QLineEdit *xCoordLineEdit = new QLineEdit;
    QLineEdit *yCoordLineEdit = new QLineEdit;

    // Adjust the size of LineEdit widgets
    xCoordLineEdit->setFixedWidth(500);
    yCoordLineEdit->setFixedWidth(500);

    QObject::connect(xCoordLineEdit, &QLineEdit::textChanged, joypad, &Joypad::setGoalX);
    QObject::connect(yCoordLineEdit, &QLineEdit::textChanged, joypad, &Joypad::setGoalY);


    QHBoxLayout *xCoordLayout = new QHBoxLayout;
    xCoordLayout->addWidget(xCoordLabel);
    xCoordLayout->addSpacing(10); // Add spacing between the label and LineEdit widget
    xCoordLayout->addWidget(xCoordLineEdit);
    xCoordLayout->addStretch(); // Add a stretch to push the LineEdit closer to the center

    QHBoxLayout *yCoordLayout = new QHBoxLayout;
    yCoordLayout->addWidget(yCoordLabel);
    yCoordLayout->addSpacing(10); // Add spacing between the label and LineEdit widget
    yCoordLayout->addWidget(yCoordLineEdit);
    yCoordLayout->addStretch(); // Add a stretch to push the LineEdit closer to the center

    layout->addLayout(xCoordLayout); // Add the X coordinate layout to the main layout
    layout->addLayout(yCoordLayout); // Add the Y coordinate layout to the main layout

    // Create a layout for the Send button and center it horizontally
    QHBoxLayout *sendButtonLayout = new QHBoxLayout;
    sendButtonLayout->addStretch(); // Add stretch to push the button to the center horizontally
    sendButtonLayout->addWidget(joypad->sendButton);
    sendButtonLayout->addStretch(); // Add stretch to push the button to the center horizontally

    layout->addLayout(sendButtonLayout); // Add the centered Send button layout to the main layout

    // Add a spacer to push the Joypad and label to the bottom
    layout->addStretch();

    // Add the Joypad and label
    layout->addWidget(joypad, 0, Qt::AlignCenter); // Add the Joypad to the layout, align it to the center
    QLabel *joypadLabel = new QLabel("Robot Joypad");
    joypadLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(joypadLabel); // Add the Joypad label to the layout

    // Add the knobs layout
    createKnobsLayout(parent, joypad);

    // Apply gradient background to the main window
    QPalette windowPalette;
    windowPalette.setBrush(QPalette::Window, QBrush(QColor(250, 250, 250))); // Set gradient background color
    parent->setPalette(windowPalette);

    parent->resize(600, 800); // Resize the window
    parent->show();
}



int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Set the application style to Fusion
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    // Set the color palette for the application
    QPalette palette;
    palette.setColor(QPalette::WindowText, Qt::black); // Set text color to black
    palette.setColor(QPalette::Button, QColor(200, 200, 200)); // Set button color to gray
    palette.setColor(QPalette::Highlight, QColor(85, 170, 255)); // Set highlight color to blue
    palette.setColor(QPalette::ButtonText, Qt::black); // Set button text color to black
    palette.setColor(QPalette::Disabled, QPalette::ButtonText, Qt::darkGray); // Set disabled button text color to dark gray
    app.setPalette(palette);

    QWidget window;
    createJoypadLayout(&window);

    return app.exec();
}

#include "main.moc"
