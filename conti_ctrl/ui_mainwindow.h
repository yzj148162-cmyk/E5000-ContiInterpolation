/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCharts/QChartView>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *rootLayout;
    QGroupBox *globalControlGroup;
    QVBoxLayout *globalControlLayout;
    QGroupBox *globalStatusGroup;
    QFormLayout *statusForm;
    QLabel *boardLabel;
    QLabel *boardValueLabel;
    QLabel *stateLabel;
    QLabel *stateValueLabel;
    QLabel *runStateLabel;
    QLabel *runStateValueLabel;
    QLabel *markLabel;
    QLabel *markValueLabel;
    QLabel *bufferLabel;
    QLabel *bufferValueLabel;
    QLabel *spaceLabel;
    QLabel *spaceValueLabel;
    QLabel *hostQueueLabel;
    QLabel *hostQueueValueLabel;
    QGroupBox *globalActionGroup;
    QVBoxLayout *globalActionLayout;
    QPushButton *initializeButton;
    QPushButton *closeBoardButton;
    QPushButton *enableAxesButton;
    QPushButton *disableAxesButton;
    QGroupBox *globalEmergencyGroup;
    QVBoxLayout *globalEmergencyLayout;
    QPushButton *emergencyStopButton;
    QLabel *globalEmergencyHintLabel;
    QSpacerItem *globalControlSpacer;
    QSplitter *mainSplitter;
    QTabWidget *tabWidget;
    QWidget *contiTestTab;
    QVBoxLayout *contiTestLayout;
    QHBoxLayout *settingsLayout;
    QGroupBox *testGroup;
    QFormLayout *testForm;
    QLabel *stageLabel;
    QComboBox *stageCombo;
    QLabel *stageHintLabel;
    QLabel *cardLabel;
    QSpinBox *cardSpin;
    QLabel *crdLabel;
    QSpinBox *crdSpin;
    QLabel *activeAxisLabel;
    QComboBox *activeAxisCombo;
    QLabel *holdAxisLabel;
    QComboBox *holdAxisCombo;
    QLabel *activeDeltaLabel;
    QDoubleSpinBox *activeDeltaSpin;
    QLabel *holdDeltaLabel;
    QDoubleSpinBox *holdDeltaSpin;
    QLabel *durationLabel;
    QDoubleSpinBox *durationSpin;
    QLabel *producerPeriodLabel;
    QSpinBox *producerPeriodSpin;
    QGroupBox *contiGroup;
    QFormLayout *contiForm;
    QLabel *maxVelocityLabel;
    QDoubleSpinBox *maxVelocitySpin;
    QLabel *accelerationLabel;
    QDoubleSpinBox *accelerationSpin;
    QLabel *decelerationLabel;
    QDoubleSpinBox *decelerationSpin;
    QLabel *sCurveLabel;
    QDoubleSpinBox *sCurveSpin;
    QLabel *speedRatioLabel;
    QDoubleSpinBox *speedRatioSpin;
    QLabel *lookaheadLabel;
    QCheckBox *lookaheadCheck;
    QLabel *pathErrorLabel;
    QDoubleSpinBox *pathErrorSpin;
    QLabel *preloadSegmentsLabel;
    QSpinBox *preloadSegmentsSpin;
    QLabel *targetBufferLabel;
    QSpinBox *targetBufferSpin;
    QLabel *lowBufferLabel;
    QSpinBox *lowBufferSpin;
    QHBoxLayout *contiButtonLayout;
    QPushButton *startButton;
    QPushButton *stopButton;
    QWidget *axisFeedbackTab;
    QVBoxLayout *axisFeedbackLayout;
    QGroupBox *baseConfigGroup;
    QFormLayout *baseConfigForm;
    QLabel *axisRangeLabel;
    QLabel *axisRangeValueLabel;
    QLabel *equivLabel;
    QLabel *equivValueLabel;
    QLabel *feedbackSourceLabel;
    QLabel *feedbackSourceValueLabel;
    QGroupBox *singleAxisJogGroup;
    QGridLayout *singleAxisJogLayout;
    QLabel *jogHintLabel;
    QLabel *jogAxisLabel;
    QComboBox *jogAxisCombo;
    QLabel *jogActualPositionLabel;
    QDoubleSpinBox *jogActualPositionSpin;
    QCheckBox *jogShowAbsoluteCheck;
    QLabel *jogTargetPositionLabel;
    QDoubleSpinBox *jogTargetPositionSpin;
    QLabel *jogVelocityLabel;
    QDoubleSpinBox *jogVelocitySpin;
    QHBoxLayout *jogButtonLayout;
    QPushButton *jogEnableButton;
    QPushButton *jogDisableButton;
    QPushButton *jogUseActualPositionButton;
    QPushButton *jogSetCurrentZeroButton;
    QPushButton *jogStartButton;
    QPushButton *jogStopButton;
    QPushButton *jogEmergencyStopButton;
    QSpacerItem *jogButtonSpacer;
    QHBoxLayout *feedbackToolbarLayout;
    QPushButton *refreshFeedbackButton;
    QLabel *feedbackSummaryValueLabel;
    QSpacerItem *feedbackToolbarSpacer;
    QTableWidget *axisFeedbackTable;
    QWidget *telemetryTab;
    QVBoxLayout *telemetryLayout;
    QGroupBox *telemetryRecordGroup;
    QGridLayout *telemetryRecordLayout;
    QPushButton *startRecordingButton;
    QPushButton *stopRecordingButton;
    QLabel *recordingStateValueLabel;
    QLabel *recordPathLabel;
    QLabel *recordPathValueLabel;
    QLabel *recordStatsLabel;
    QLabel *recordStatsValueLabel;
    QSplitter *telemetryPlotSplitter;
    QChartView *positionChartView;
    QChartView *followingErrorChartView;
    QLabel *telemetryHintLabel;
    QWidget *extensionTab;
    QVBoxLayout *extensionLayout;
    QLabel *extensionHintLabel;
    QSpacerItem *extensionVerticalSpacer;
    QGroupBox *logGroup;
    QVBoxLayout *logLayout;
    QHBoxLayout *logToolbarLayout;
    QLabel *logHintLabel;
    QSpacerItem *logToolbarSpacer;
    QPushButton *clearLogButton;
    QPlainTextEdit *logEdit;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1440, 820);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        rootLayout = new QHBoxLayout(centralwidget);
        rootLayout->setObjectName("rootLayout");
        globalControlGroup = new QGroupBox(centralwidget);
        globalControlGroup->setObjectName("globalControlGroup");
        globalControlGroup->setMinimumSize(QSize(230, 0));
        globalControlGroup->setMaximumSize(QSize(300, 16777215));
        globalControlLayout = new QVBoxLayout(globalControlGroup);
        globalControlLayout->setObjectName("globalControlLayout");
        globalStatusGroup = new QGroupBox(globalControlGroup);
        globalStatusGroup->setObjectName("globalStatusGroup");
        statusForm = new QFormLayout(globalStatusGroup);
        statusForm->setObjectName("statusForm");
        boardLabel = new QLabel(globalStatusGroup);
        boardLabel->setObjectName("boardLabel");

        statusForm->setWidget(0, QFormLayout::LabelRole, boardLabel);

        boardValueLabel = new QLabel(globalStatusGroup);
        boardValueLabel->setObjectName("boardValueLabel");
        boardValueLabel->setWordWrap(true);

        statusForm->setWidget(0, QFormLayout::FieldRole, boardValueLabel);

        stateLabel = new QLabel(globalStatusGroup);
        stateLabel->setObjectName("stateLabel");

        statusForm->setWidget(1, QFormLayout::LabelRole, stateLabel);

        stateValueLabel = new QLabel(globalStatusGroup);
        stateValueLabel->setObjectName("stateValueLabel");
        stateValueLabel->setWordWrap(true);

        statusForm->setWidget(1, QFormLayout::FieldRole, stateValueLabel);

        runStateLabel = new QLabel(globalStatusGroup);
        runStateLabel->setObjectName("runStateLabel");

        statusForm->setWidget(2, QFormLayout::LabelRole, runStateLabel);

        runStateValueLabel = new QLabel(globalStatusGroup);
        runStateValueLabel->setObjectName("runStateValueLabel");

        statusForm->setWidget(2, QFormLayout::FieldRole, runStateValueLabel);

        markLabel = new QLabel(globalStatusGroup);
        markLabel->setObjectName("markLabel");
        markLabel->setWordWrap(true);

        statusForm->setWidget(3, QFormLayout::LabelRole, markLabel);

        markValueLabel = new QLabel(globalStatusGroup);
        markValueLabel->setObjectName("markValueLabel");

        statusForm->setWidget(3, QFormLayout::FieldRole, markValueLabel);

        bufferLabel = new QLabel(globalStatusGroup);
        bufferLabel->setObjectName("bufferLabel");

        statusForm->setWidget(4, QFormLayout::LabelRole, bufferLabel);

        bufferValueLabel = new QLabel(globalStatusGroup);
        bufferValueLabel->setObjectName("bufferValueLabel");

        statusForm->setWidget(4, QFormLayout::FieldRole, bufferValueLabel);

        spaceLabel = new QLabel(globalStatusGroup);
        spaceLabel->setObjectName("spaceLabel");
        spaceLabel->setWordWrap(true);

        statusForm->setWidget(5, QFormLayout::LabelRole, spaceLabel);

        spaceValueLabel = new QLabel(globalStatusGroup);
        spaceValueLabel->setObjectName("spaceValueLabel");

        statusForm->setWidget(5, QFormLayout::FieldRole, spaceValueLabel);

        hostQueueLabel = new QLabel(globalStatusGroup);
        hostQueueLabel->setObjectName("hostQueueLabel");

        statusForm->setWidget(6, QFormLayout::LabelRole, hostQueueLabel);

        hostQueueValueLabel = new QLabel(globalStatusGroup);
        hostQueueValueLabel->setObjectName("hostQueueValueLabel");

        statusForm->setWidget(6, QFormLayout::FieldRole, hostQueueValueLabel);


        globalControlLayout->addWidget(globalStatusGroup);

        globalActionGroup = new QGroupBox(globalControlGroup);
        globalActionGroup->setObjectName("globalActionGroup");
        globalActionLayout = new QVBoxLayout(globalActionGroup);
        globalActionLayout->setObjectName("globalActionLayout");
        initializeButton = new QPushButton(globalActionGroup);
        initializeButton->setObjectName("initializeButton");

        globalActionLayout->addWidget(initializeButton);

        closeBoardButton = new QPushButton(globalActionGroup);
        closeBoardButton->setObjectName("closeBoardButton");

        globalActionLayout->addWidget(closeBoardButton);

        enableAxesButton = new QPushButton(globalActionGroup);
        enableAxesButton->setObjectName("enableAxesButton");

        globalActionLayout->addWidget(enableAxesButton);

        disableAxesButton = new QPushButton(globalActionGroup);
        disableAxesButton->setObjectName("disableAxesButton");

        globalActionLayout->addWidget(disableAxesButton);


        globalControlLayout->addWidget(globalActionGroup);

        globalEmergencyGroup = new QGroupBox(globalControlGroup);
        globalEmergencyGroup->setObjectName("globalEmergencyGroup");
        globalEmergencyLayout = new QVBoxLayout(globalEmergencyGroup);
        globalEmergencyLayout->setObjectName("globalEmergencyLayout");
        emergencyStopButton = new QPushButton(globalEmergencyGroup);
        emergencyStopButton->setObjectName("emergencyStopButton");
        emergencyStopButton->setMinimumSize(QSize(0, 42));
        emergencyStopButton->setStyleSheet(QString::fromUtf8("color: white; background-color: #b00020; font-weight: bold;"));

        globalEmergencyLayout->addWidget(emergencyStopButton);

        globalEmergencyHintLabel = new QLabel(globalEmergencyGroup);
        globalEmergencyHintLabel->setObjectName("globalEmergencyHintLabel");
        globalEmergencyHintLabel->setWordWrap(true);

        globalEmergencyLayout->addWidget(globalEmergencyHintLabel);


        globalControlLayout->addWidget(globalEmergencyGroup);

        globalControlSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        globalControlLayout->addItem(globalControlSpacer);


        rootLayout->addWidget(globalControlGroup);

        mainSplitter = new QSplitter(centralwidget);
        mainSplitter->setObjectName("mainSplitter");
        mainSplitter->setOrientation(Qt::Orientation::Horizontal);
        mainSplitter->setChildrenCollapsible(false);
        tabWidget = new QTabWidget(mainSplitter);
        tabWidget->setObjectName("tabWidget");
        contiTestTab = new QWidget();
        contiTestTab->setObjectName("contiTestTab");
        contiTestLayout = new QVBoxLayout(contiTestTab);
        contiTestLayout->setObjectName("contiTestLayout");
        settingsLayout = new QHBoxLayout();
        settingsLayout->setObjectName("settingsLayout");
        testGroup = new QGroupBox(contiTestTab);
        testGroup->setObjectName("testGroup");
        testForm = new QFormLayout(testGroup);
        testForm->setObjectName("testForm");
        stageLabel = new QLabel(testGroup);
        stageLabel->setObjectName("stageLabel");

        testForm->setWidget(0, QFormLayout::LabelRole, stageLabel);

        stageCombo = new QComboBox(testGroup);
        stageCombo->addItem(QString());
        stageCombo->addItem(QString());
        stageCombo->setObjectName("stageCombo");

        testForm->setWidget(0, QFormLayout::FieldRole, stageCombo);

        stageHintLabel = new QLabel(testGroup);
        stageHintLabel->setObjectName("stageHintLabel");
        stageHintLabel->setWordWrap(true);

        testForm->setWidget(1, QFormLayout::SpanningRole, stageHintLabel);

        cardLabel = new QLabel(testGroup);
        cardLabel->setObjectName("cardLabel");

        testForm->setWidget(2, QFormLayout::LabelRole, cardLabel);

        cardSpin = new QSpinBox(testGroup);
        cardSpin->setObjectName("cardSpin");
        cardSpin->setMaximum(15);

        testForm->setWidget(2, QFormLayout::FieldRole, cardSpin);

        crdLabel = new QLabel(testGroup);
        crdLabel->setObjectName("crdLabel");

        testForm->setWidget(3, QFormLayout::LabelRole, crdLabel);

        crdSpin = new QSpinBox(testGroup);
        crdSpin->setObjectName("crdSpin");
        crdSpin->setMaximum(7);

        testForm->setWidget(3, QFormLayout::FieldRole, crdSpin);

        activeAxisLabel = new QLabel(testGroup);
        activeAxisLabel->setObjectName("activeAxisLabel");

        testForm->setWidget(4, QFormLayout::LabelRole, activeAxisLabel);

        activeAxisCombo = new QComboBox(testGroup);
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->addItem(QString());
        activeAxisCombo->setObjectName("activeAxisCombo");

        testForm->setWidget(4, QFormLayout::FieldRole, activeAxisCombo);

        holdAxisLabel = new QLabel(testGroup);
        holdAxisLabel->setObjectName("holdAxisLabel");

        testForm->setWidget(5, QFormLayout::LabelRole, holdAxisLabel);

        holdAxisCombo = new QComboBox(testGroup);
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->addItem(QString());
        holdAxisCombo->setObjectName("holdAxisCombo");

        testForm->setWidget(5, QFormLayout::FieldRole, holdAxisCombo);

        activeDeltaLabel = new QLabel(testGroup);
        activeDeltaLabel->setObjectName("activeDeltaLabel");

        testForm->setWidget(6, QFormLayout::LabelRole, activeDeltaLabel);

        activeDeltaSpin = new QDoubleSpinBox(testGroup);
        activeDeltaSpin->setObjectName("activeDeltaSpin");
        activeDeltaSpin->setDecimals(4);
        activeDeltaSpin->setMinimum(-1000000.000000000000000);
        activeDeltaSpin->setMaximum(1000000.000000000000000);
        activeDeltaSpin->setValue(1.000000000000000);

        testForm->setWidget(6, QFormLayout::FieldRole, activeDeltaSpin);

        holdDeltaLabel = new QLabel(testGroup);
        holdDeltaLabel->setObjectName("holdDeltaLabel");

        testForm->setWidget(7, QFormLayout::LabelRole, holdDeltaLabel);

        holdDeltaSpin = new QDoubleSpinBox(testGroup);
        holdDeltaSpin->setObjectName("holdDeltaSpin");
        holdDeltaSpin->setDecimals(4);
        holdDeltaSpin->setMinimum(-1000000.000000000000000);
        holdDeltaSpin->setMaximum(1000000.000000000000000);
        holdDeltaSpin->setValue(1.000000000000000);

        testForm->setWidget(7, QFormLayout::FieldRole, holdDeltaSpin);

        durationLabel = new QLabel(testGroup);
        durationLabel->setObjectName("durationLabel");

        testForm->setWidget(8, QFormLayout::LabelRole, durationLabel);

        durationSpin = new QDoubleSpinBox(testGroup);
        durationSpin->setObjectName("durationSpin");
        durationSpin->setDecimals(3);
        durationSpin->setMinimum(0.100000000000000);
        durationSpin->setMaximum(3600.000000000000000);
        durationSpin->setValue(2.000000000000000);

        testForm->setWidget(8, QFormLayout::FieldRole, durationSpin);

        producerPeriodLabel = new QLabel(testGroup);
        producerPeriodLabel->setObjectName("producerPeriodLabel");

        testForm->setWidget(9, QFormLayout::LabelRole, producerPeriodLabel);

        producerPeriodSpin = new QSpinBox(testGroup);
        producerPeriodSpin->setObjectName("producerPeriodSpin");
        producerPeriodSpin->setMinimum(1);
        producerPeriodSpin->setMaximum(1000);
        producerPeriodSpin->setValue(10);

        testForm->setWidget(9, QFormLayout::FieldRole, producerPeriodSpin);


        settingsLayout->addWidget(testGroup);

        contiGroup = new QGroupBox(contiTestTab);
        contiGroup->setObjectName("contiGroup");
        contiForm = new QFormLayout(contiGroup);
        contiForm->setObjectName("contiForm");
        maxVelocityLabel = new QLabel(contiGroup);
        maxVelocityLabel->setObjectName("maxVelocityLabel");

        contiForm->setWidget(0, QFormLayout::LabelRole, maxVelocityLabel);

        maxVelocitySpin = new QDoubleSpinBox(contiGroup);
        maxVelocitySpin->setObjectName("maxVelocitySpin");
        maxVelocitySpin->setDecimals(4);
        maxVelocitySpin->setMinimum(0.000100000000000);
        maxVelocitySpin->setMaximum(100000000.000000000000000);
        maxVelocitySpin->setValue(10.000000000000000);

        contiForm->setWidget(0, QFormLayout::FieldRole, maxVelocitySpin);

        accelerationLabel = new QLabel(contiGroup);
        accelerationLabel->setObjectName("accelerationLabel");

        contiForm->setWidget(1, QFormLayout::LabelRole, accelerationLabel);

        accelerationSpin = new QDoubleSpinBox(contiGroup);
        accelerationSpin->setObjectName("accelerationSpin");
        accelerationSpin->setDecimals(3);
        accelerationSpin->setMinimum(0.001000000000000);
        accelerationSpin->setMaximum(60.000000000000000);
        accelerationSpin->setValue(0.050000000000000);

        contiForm->setWidget(1, QFormLayout::FieldRole, accelerationSpin);

        decelerationLabel = new QLabel(contiGroup);
        decelerationLabel->setObjectName("decelerationLabel");

        contiForm->setWidget(2, QFormLayout::LabelRole, decelerationLabel);

        decelerationSpin = new QDoubleSpinBox(contiGroup);
        decelerationSpin->setObjectName("decelerationSpin");
        decelerationSpin->setDecimals(3);
        decelerationSpin->setMinimum(0.001000000000000);
        decelerationSpin->setMaximum(60.000000000000000);
        decelerationSpin->setValue(0.050000000000000);

        contiForm->setWidget(2, QFormLayout::FieldRole, decelerationSpin);

        sCurveLabel = new QLabel(contiGroup);
        sCurveLabel->setObjectName("sCurveLabel");

        contiForm->setWidget(3, QFormLayout::LabelRole, sCurveLabel);

        sCurveSpin = new QDoubleSpinBox(contiGroup);
        sCurveSpin->setObjectName("sCurveSpin");
        sCurveSpin->setDecimals(3);
        sCurveSpin->setMinimum(0.000000000000000);
        sCurveSpin->setMaximum(60.000000000000000);
        sCurveSpin->setValue(0.020000000000000);

        contiForm->setWidget(3, QFormLayout::FieldRole, sCurveSpin);

        speedRatioLabel = new QLabel(contiGroup);
        speedRatioLabel->setObjectName("speedRatioLabel");

        contiForm->setWidget(4, QFormLayout::LabelRole, speedRatioLabel);

        speedRatioSpin = new QDoubleSpinBox(contiGroup);
        speedRatioSpin->setObjectName("speedRatioSpin");
        speedRatioSpin->setDecimals(2);
        speedRatioSpin->setMinimum(0.010000000000000);
        speedRatioSpin->setMaximum(1.000000000000000);
        speedRatioSpin->setSingleStep(0.050000000000000);
        speedRatioSpin->setValue(0.500000000000000);

        contiForm->setWidget(4, QFormLayout::FieldRole, speedRatioSpin);

        lookaheadLabel = new QLabel(contiGroup);
        lookaheadLabel->setObjectName("lookaheadLabel");

        contiForm->setWidget(5, QFormLayout::LabelRole, lookaheadLabel);

        lookaheadCheck = new QCheckBox(contiGroup);
        lookaheadCheck->setObjectName("lookaheadCheck");
        lookaheadCheck->setChecked(true);

        contiForm->setWidget(5, QFormLayout::FieldRole, lookaheadCheck);

        pathErrorLabel = new QLabel(contiGroup);
        pathErrorLabel->setObjectName("pathErrorLabel");

        contiForm->setWidget(6, QFormLayout::LabelRole, pathErrorLabel);

        pathErrorSpin = new QDoubleSpinBox(contiGroup);
        pathErrorSpin->setObjectName("pathErrorSpin");
        pathErrorSpin->setDecimals(4);
        pathErrorSpin->setMaximum(1000000.000000000000000);

        contiForm->setWidget(6, QFormLayout::FieldRole, pathErrorSpin);

        preloadSegmentsLabel = new QLabel(contiGroup);
        preloadSegmentsLabel->setObjectName("preloadSegmentsLabel");

        contiForm->setWidget(7, QFormLayout::LabelRole, preloadSegmentsLabel);

        preloadSegmentsSpin = new QSpinBox(contiGroup);
        preloadSegmentsSpin->setObjectName("preloadSegmentsSpin");
        preloadSegmentsSpin->setMinimum(2);
        preloadSegmentsSpin->setMaximum(1000);
        preloadSegmentsSpin->setValue(8);

        contiForm->setWidget(7, QFormLayout::FieldRole, preloadSegmentsSpin);

        targetBufferLabel = new QLabel(contiGroup);
        targetBufferLabel->setObjectName("targetBufferLabel");

        contiForm->setWidget(8, QFormLayout::LabelRole, targetBufferLabel);

        targetBufferSpin = new QSpinBox(contiGroup);
        targetBufferSpin->setObjectName("targetBufferSpin");
        targetBufferSpin->setMinimum(2);
        targetBufferSpin->setMaximum(1000);
        targetBufferSpin->setValue(8);

        contiForm->setWidget(8, QFormLayout::FieldRole, targetBufferSpin);

        lowBufferLabel = new QLabel(contiGroup);
        lowBufferLabel->setObjectName("lowBufferLabel");

        contiForm->setWidget(9, QFormLayout::LabelRole, lowBufferLabel);

        lowBufferSpin = new QSpinBox(contiGroup);
        lowBufferSpin->setObjectName("lowBufferSpin");
        lowBufferSpin->setMinimum(1);
        lowBufferSpin->setMaximum(999);
        lowBufferSpin->setValue(4);

        contiForm->setWidget(9, QFormLayout::FieldRole, lowBufferSpin);


        settingsLayout->addWidget(contiGroup);


        contiTestLayout->addLayout(settingsLayout);

        contiButtonLayout = new QHBoxLayout();
        contiButtonLayout->setObjectName("contiButtonLayout");
        startButton = new QPushButton(contiTestTab);
        startButton->setObjectName("startButton");

        contiButtonLayout->addWidget(startButton);

        stopButton = new QPushButton(contiTestTab);
        stopButton->setObjectName("stopButton");

        contiButtonLayout->addWidget(stopButton);


        contiTestLayout->addLayout(contiButtonLayout);

        tabWidget->addTab(contiTestTab, QString());
        axisFeedbackTab = new QWidget();
        axisFeedbackTab->setObjectName("axisFeedbackTab");
        axisFeedbackLayout = new QVBoxLayout(axisFeedbackTab);
        axisFeedbackLayout->setObjectName("axisFeedbackLayout");
        baseConfigGroup = new QGroupBox(axisFeedbackTab);
        baseConfigGroup->setObjectName("baseConfigGroup");
        baseConfigForm = new QFormLayout(baseConfigGroup);
        baseConfigForm->setObjectName("baseConfigForm");
        axisRangeLabel = new QLabel(baseConfigGroup);
        axisRangeLabel->setObjectName("axisRangeLabel");

        baseConfigForm->setWidget(0, QFormLayout::LabelRole, axisRangeLabel);

        axisRangeValueLabel = new QLabel(baseConfigGroup);
        axisRangeValueLabel->setObjectName("axisRangeValueLabel");

        baseConfigForm->setWidget(0, QFormLayout::FieldRole, axisRangeValueLabel);

        equivLabel = new QLabel(baseConfigGroup);
        equivLabel->setObjectName("equivLabel");

        baseConfigForm->setWidget(1, QFormLayout::LabelRole, equivLabel);

        equivValueLabel = new QLabel(baseConfigGroup);
        equivValueLabel->setObjectName("equivValueLabel");

        baseConfigForm->setWidget(1, QFormLayout::FieldRole, equivValueLabel);

        feedbackSourceLabel = new QLabel(baseConfigGroup);
        feedbackSourceLabel->setObjectName("feedbackSourceLabel");

        baseConfigForm->setWidget(2, QFormLayout::LabelRole, feedbackSourceLabel);

        feedbackSourceValueLabel = new QLabel(baseConfigGroup);
        feedbackSourceValueLabel->setObjectName("feedbackSourceValueLabel");

        baseConfigForm->setWidget(2, QFormLayout::FieldRole, feedbackSourceValueLabel);


        axisFeedbackLayout->addWidget(baseConfigGroup);

        singleAxisJogGroup = new QGroupBox(axisFeedbackTab);
        singleAxisJogGroup->setObjectName("singleAxisJogGroup");
        singleAxisJogLayout = new QGridLayout(singleAxisJogGroup);
        singleAxisJogLayout->setObjectName("singleAxisJogLayout");
        jogHintLabel = new QLabel(singleAxisJogGroup);
        jogHintLabel->setObjectName("jogHintLabel");
        jogHintLabel->setWordWrap(true);

        singleAxisJogLayout->addWidget(jogHintLabel, 0, 0, 1, 4);

        jogAxisLabel = new QLabel(singleAxisJogGroup);
        jogAxisLabel->setObjectName("jogAxisLabel");

        singleAxisJogLayout->addWidget(jogAxisLabel, 1, 0, 1, 1);

        jogAxisCombo = new QComboBox(singleAxisJogGroup);
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->addItem(QString());
        jogAxisCombo->setObjectName("jogAxisCombo");

        singleAxisJogLayout->addWidget(jogAxisCombo, 1, 1, 1, 1);

        jogActualPositionLabel = new QLabel(singleAxisJogGroup);
        jogActualPositionLabel->setObjectName("jogActualPositionLabel");

        singleAxisJogLayout->addWidget(jogActualPositionLabel, 1, 2, 1, 1);

        jogActualPositionSpin = new QDoubleSpinBox(singleAxisJogGroup);
        jogActualPositionSpin->setObjectName("jogActualPositionSpin");
        jogActualPositionSpin->setReadOnly(true);
        jogActualPositionSpin->setDecimals(4);
        jogActualPositionSpin->setMinimum(-1000000.000000000000000);
        jogActualPositionSpin->setMaximum(1000000.000000000000000);

        singleAxisJogLayout->addWidget(jogActualPositionSpin, 1, 3, 1, 1);

        jogShowAbsoluteCheck = new QCheckBox(singleAxisJogGroup);
        jogShowAbsoluteCheck->setObjectName("jogShowAbsoluteCheck");

        singleAxisJogLayout->addWidget(jogShowAbsoluteCheck, 1, 4, 1, 1);

        jogTargetPositionLabel = new QLabel(singleAxisJogGroup);
        jogTargetPositionLabel->setObjectName("jogTargetPositionLabel");

        singleAxisJogLayout->addWidget(jogTargetPositionLabel, 2, 0, 1, 1);

        jogTargetPositionSpin = new QDoubleSpinBox(singleAxisJogGroup);
        jogTargetPositionSpin->setObjectName("jogTargetPositionSpin");
        jogTargetPositionSpin->setDecimals(4);
        jogTargetPositionSpin->setMinimum(-1000000.000000000000000);
        jogTargetPositionSpin->setMaximum(1000000.000000000000000);

        singleAxisJogLayout->addWidget(jogTargetPositionSpin, 2, 1, 1, 1);

        jogVelocityLabel = new QLabel(singleAxisJogGroup);
        jogVelocityLabel->setObjectName("jogVelocityLabel");

        singleAxisJogLayout->addWidget(jogVelocityLabel, 2, 2, 1, 1);

        jogVelocitySpin = new QDoubleSpinBox(singleAxisJogGroup);
        jogVelocitySpin->setObjectName("jogVelocitySpin");
        jogVelocitySpin->setDecimals(4);
        jogVelocitySpin->setMinimum(0.000100000000000);
        jogVelocitySpin->setMaximum(1000000.000000000000000);
        jogVelocitySpin->setValue(10.000000000000000);

        singleAxisJogLayout->addWidget(jogVelocitySpin, 2, 3, 1, 1);

        jogButtonLayout = new QHBoxLayout();
        jogButtonLayout->setObjectName("jogButtonLayout");
        jogEnableButton = new QPushButton(singleAxisJogGroup);
        jogEnableButton->setObjectName("jogEnableButton");

        jogButtonLayout->addWidget(jogEnableButton);

        jogDisableButton = new QPushButton(singleAxisJogGroup);
        jogDisableButton->setObjectName("jogDisableButton");

        jogButtonLayout->addWidget(jogDisableButton);

        jogUseActualPositionButton = new QPushButton(singleAxisJogGroup);
        jogUseActualPositionButton->setObjectName("jogUseActualPositionButton");

        jogButtonLayout->addWidget(jogUseActualPositionButton);

        jogSetCurrentZeroButton = new QPushButton(singleAxisJogGroup);
        jogSetCurrentZeroButton->setObjectName("jogSetCurrentZeroButton");

        jogButtonLayout->addWidget(jogSetCurrentZeroButton);

        jogStartButton = new QPushButton(singleAxisJogGroup);
        jogStartButton->setObjectName("jogStartButton");

        jogButtonLayout->addWidget(jogStartButton);

        jogStopButton = new QPushButton(singleAxisJogGroup);
        jogStopButton->setObjectName("jogStopButton");

        jogButtonLayout->addWidget(jogStopButton);

        jogEmergencyStopButton = new QPushButton(singleAxisJogGroup);
        jogEmergencyStopButton->setObjectName("jogEmergencyStopButton");
        jogEmergencyStopButton->setStyleSheet(QString::fromUtf8("color: white; background-color: #b02020;"));

        jogButtonLayout->addWidget(jogEmergencyStopButton);

        jogButtonSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        jogButtonLayout->addItem(jogButtonSpacer);


        singleAxisJogLayout->addLayout(jogButtonLayout, 3, 0, 1, 4);


        axisFeedbackLayout->addWidget(singleAxisJogGroup);

        feedbackToolbarLayout = new QHBoxLayout();
        feedbackToolbarLayout->setObjectName("feedbackToolbarLayout");
        refreshFeedbackButton = new QPushButton(axisFeedbackTab);
        refreshFeedbackButton->setObjectName("refreshFeedbackButton");

        feedbackToolbarLayout->addWidget(refreshFeedbackButton);

        feedbackSummaryValueLabel = new QLabel(axisFeedbackTab);
        feedbackSummaryValueLabel->setObjectName("feedbackSummaryValueLabel");

        feedbackToolbarLayout->addWidget(feedbackSummaryValueLabel);

        feedbackToolbarSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        feedbackToolbarLayout->addItem(feedbackToolbarSpacer);


        axisFeedbackLayout->addLayout(feedbackToolbarLayout);

        axisFeedbackTable = new QTableWidget(axisFeedbackTab);
        if (axisFeedbackTable->columnCount() < 8)
            axisFeedbackTable->setColumnCount(8);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(6, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        axisFeedbackTable->setHorizontalHeaderItem(7, __qtablewidgetitem7);
        if (axisFeedbackTable->rowCount() < 8)
            axisFeedbackTable->setRowCount(8);
        axisFeedbackTable->setObjectName("axisFeedbackTable");
        axisFeedbackTable->setEditTriggers(QAbstractItemView::EditTrigger::NoEditTriggers);
        axisFeedbackTable->setAlternatingRowColors(true);
        axisFeedbackTable->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
        axisFeedbackTable->setRowCount(8);
        axisFeedbackTable->setColumnCount(8);

        axisFeedbackLayout->addWidget(axisFeedbackTable);

        tabWidget->addTab(axisFeedbackTab, QString());
        telemetryTab = new QWidget();
        telemetryTab->setObjectName("telemetryTab");
        telemetryLayout = new QVBoxLayout(telemetryTab);
        telemetryLayout->setObjectName("telemetryLayout");
        telemetryRecordGroup = new QGroupBox(telemetryTab);
        telemetryRecordGroup->setObjectName("telemetryRecordGroup");
        telemetryRecordLayout = new QGridLayout(telemetryRecordGroup);
        telemetryRecordLayout->setObjectName("telemetryRecordLayout");
        startRecordingButton = new QPushButton(telemetryRecordGroup);
        startRecordingButton->setObjectName("startRecordingButton");

        telemetryRecordLayout->addWidget(startRecordingButton, 0, 0, 1, 1);

        stopRecordingButton = new QPushButton(telemetryRecordGroup);
        stopRecordingButton->setObjectName("stopRecordingButton");

        telemetryRecordLayout->addWidget(stopRecordingButton, 0, 1, 1, 1);

        recordingStateValueLabel = new QLabel(telemetryRecordGroup);
        recordingStateValueLabel->setObjectName("recordingStateValueLabel");

        telemetryRecordLayout->addWidget(recordingStateValueLabel, 0, 2, 1, 1);

        recordPathLabel = new QLabel(telemetryRecordGroup);
        recordPathLabel->setObjectName("recordPathLabel");

        telemetryRecordLayout->addWidget(recordPathLabel, 1, 0, 1, 1);

        recordPathValueLabel = new QLabel(telemetryRecordGroup);
        recordPathValueLabel->setObjectName("recordPathValueLabel");
        recordPathValueLabel->setWordWrap(true);

        telemetryRecordLayout->addWidget(recordPathValueLabel, 1, 1, 1, 2);

        recordStatsLabel = new QLabel(telemetryRecordGroup);
        recordStatsLabel->setObjectName("recordStatsLabel");

        telemetryRecordLayout->addWidget(recordStatsLabel, 2, 0, 1, 1);

        recordStatsValueLabel = new QLabel(telemetryRecordGroup);
        recordStatsValueLabel->setObjectName("recordStatsValueLabel");

        telemetryRecordLayout->addWidget(recordStatsValueLabel, 2, 1, 1, 2);


        telemetryLayout->addWidget(telemetryRecordGroup);

        telemetryPlotSplitter = new QSplitter(telemetryTab);
        telemetryPlotSplitter->setObjectName("telemetryPlotSplitter");
        telemetryPlotSplitter->setOrientation(Qt::Orientation::Vertical);
        telemetryPlotSplitter->setChildrenCollapsible(false);
        positionChartView = new QChartView(telemetryPlotSplitter);
        positionChartView->setObjectName("positionChartView");
        positionChartView->setMinimumSize(QSize(0, 240));
        telemetryPlotSplitter->addWidget(positionChartView);
        followingErrorChartView = new QChartView(telemetryPlotSplitter);
        followingErrorChartView->setObjectName("followingErrorChartView");
        followingErrorChartView->setMinimumSize(QSize(0, 240));
        telemetryPlotSplitter->addWidget(followingErrorChartView);

        telemetryLayout->addWidget(telemetryPlotSplitter);

        telemetryHintLabel = new QLabel(telemetryTab);
        telemetryHintLabel->setObjectName("telemetryHintLabel");
        telemetryHintLabel->setWordWrap(true);

        telemetryLayout->addWidget(telemetryHintLabel);

        tabWidget->addTab(telemetryTab, QString());
        extensionTab = new QWidget();
        extensionTab->setObjectName("extensionTab");
        extensionLayout = new QVBoxLayout(extensionTab);
        extensionLayout->setObjectName("extensionLayout");
        extensionHintLabel = new QLabel(extensionTab);
        extensionHintLabel->setObjectName("extensionHintLabel");
        extensionHintLabel->setAlignment(Qt::AlignmentFlag::AlignLeading|Qt::AlignmentFlag::AlignLeft|Qt::AlignmentFlag::AlignTop);
        extensionHintLabel->setWordWrap(true);

        extensionLayout->addWidget(extensionHintLabel);

        extensionVerticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        extensionLayout->addItem(extensionVerticalSpacer);

        tabWidget->addTab(extensionTab, QString());
        mainSplitter->addWidget(tabWidget);
        logGroup = new QGroupBox(mainSplitter);
        logGroup->setObjectName("logGroup");
        logGroup->setMinimumSize(QSize(280, 0));
        logLayout = new QVBoxLayout(logGroup);
        logLayout->setObjectName("logLayout");
        logToolbarLayout = new QHBoxLayout();
        logToolbarLayout->setObjectName("logToolbarLayout");
        logHintLabel = new QLabel(logGroup);
        logHintLabel->setObjectName("logHintLabel");

        logToolbarLayout->addWidget(logHintLabel);

        logToolbarSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        logToolbarLayout->addItem(logToolbarSpacer);

        clearLogButton = new QPushButton(logGroup);
        clearLogButton->setObjectName("clearLogButton");

        logToolbarLayout->addWidget(clearLogButton);


        logLayout->addLayout(logToolbarLayout);

        logEdit = new QPlainTextEdit(logGroup);
        logEdit->setObjectName("logEdit");
        logEdit->setReadOnly(true);
        logEdit->setMaximumBlockCount(2000);

        logLayout->addWidget(logEdit);

        mainSplitter->addWidget(logGroup);

        rootLayout->addWidget(mainSplitter);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);
        holdAxisCombo->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "E5000 \350\277\236\347\273\255\346\217\222\350\241\245\346\265\213\350\257\225\345\231\250", nullptr));
        globalControlGroup->setTitle(QCoreApplication::translate("MainWindow", "\345\205\250\345\261\200\346\216\247\345\210\266\344\270\216\347\212\266\346\200\201", nullptr));
        globalStatusGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241\347\212\266\346\200\201", nullptr));
        boardLabel->setText(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241", nullptr));
        boardValueLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\252\345\210\235\345\247\213\345\214\226", nullptr));
        stateLabel->setText(QCoreApplication::translate("MainWindow", "\347\250\213\345\272\217\347\212\266\346\200\201", nullptr));
        stateValueLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\252\345\210\235\345\247\213\345\214\226", nullptr));
        runStateLabel->setText(QCoreApplication::translate("MainWindow", "runState", nullptr));
        runStateValueLabel->setText(QCoreApplication::translate("MainWindow", "4", nullptr));
        markLabel->setText(QCoreApplication::translate("MainWindow", "\345\275\223\345\211\215 / \345\267\262\346\216\250\351\200\201 mark", nullptr));
        markValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0", nullptr));
        bufferLabel->setText(QCoreApplication::translate("MainWindow", "\347\274\223\345\206\262\346\256\265\346\225\260", nullptr));
        bufferValueLabel->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        spaceLabel->setText(QCoreApplication::translate("MainWindow", "\347\274\223\345\206\262\345\211\251\344\275\231\347\251\272\351\227\264", nullptr));
        spaceValueLabel->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        hostQueueLabel->setText(QCoreApplication::translate("MainWindow", "\344\270\212\344\275\215\346\234\272\351\230\237\345\210\227", nullptr));
        hostQueueValueLabel->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        globalActionGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241\344\270\216\346\265\213\350\257\225\350\275\264", nullptr));
        initializeButton->setText(QCoreApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226\346\216\247\345\210\266\345\215\241", nullptr));
        closeBoardButton->setText(QCoreApplication::translate("MainWindow", "\345\256\211\345\205\250\345\205\263\351\227\255\346\216\247\345\210\266\345\215\241", nullptr));
#if QT_CONFIG(tooltip)
        enableAxesButton->setToolTip(QCoreApplication::translate("MainWindow", "\344\275\277\350\203\275\347\254\254\344\270\200\351\241\265\342\200\234\344\270\273\345\212\250\350\275\264\342\200\235\345\222\214\342\200\234\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264\342\200\235\346\211\200\351\200\211\347\232\204\346\265\213\350\257\225\350\275\264\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        enableAxesButton->setText(QCoreApplication::translate("MainWindow", "\344\275\277\350\203\275\350\277\236\347\273\255\346\217\222\350\241\245\346\265\213\350\257\225\350\275\264", nullptr));
#if QT_CONFIG(tooltip)
        disableAxesButton->setToolTip(QCoreApplication::translate("MainWindow", "\345\244\261\350\203\275\347\254\254\344\270\200\351\241\265\342\200\234\344\270\273\345\212\250\350\275\264\342\200\235\345\222\214\342\200\234\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264\342\200\235\346\211\200\351\200\211\347\232\204\346\265\213\350\257\225\350\275\264\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        disableAxesButton->setText(QCoreApplication::translate("MainWindow", "\345\244\261\350\203\275\350\277\236\347\273\255\346\217\222\350\241\245\346\265\213\350\257\225\350\275\264", nullptr));
        globalEmergencyGroup->setTitle(QCoreApplication::translate("MainWindow", "\347\264\247\346\200\245\346\223\215\344\275\234", nullptr));
        emergencyStopButton->setText(QCoreApplication::translate("MainWindow", "\345\205\250\345\261\200\347\253\213\345\215\263\345\201\234\346\255\242", nullptr));
        globalEmergencyHintLabel->setText(QCoreApplication::translate("MainWindow", "\347\253\213\345\215\263\345\201\234\346\255\242\350\277\236\347\273\255\346\217\222\350\241\245\346\210\226\346\255\243\345\234\250\346\211\247\350\241\214\347\232\204\345\215\225\350\275\264\347\202\271\344\275\215\350\277\220\345\212\250\343\200\202", nullptr));
        testGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\265\213\350\257\225\350\275\250\350\277\271\357\274\210\344\272\224\346\254\241\345\244\232\351\241\271\345\274\217\357\274\211", nullptr));
        stageLabel->setText(QCoreApplication::translate("MainWindow", "\346\265\213\350\257\225\351\230\266\346\256\265", nullptr));
        stageCombo->setItemText(0, QCoreApplication::translate("MainWindow", "\351\230\266\346\256\265\344\270\200\357\274\232\345\215\225\350\275\264\350\277\220\345\212\250", nullptr));
        stageCombo->setItemText(1, QCoreApplication::translate("MainWindow", "\351\230\266\346\256\265\344\272\214\357\274\232\345\217\214\350\275\264\345\220\214\346\255\245", nullptr));

        cardLabel->setText(QCoreApplication::translate("MainWindow", "\345\215\241\345\217\267", nullptr));
        crdLabel->setText(QCoreApplication::translate("MainWindow", "\345\235\220\346\240\207\347\263\273", nullptr));
        activeAxisLabel->setText(QCoreApplication::translate("MainWindow", "\344\270\273\345\212\250\350\275\264", nullptr));
        activeAxisCombo->setItemText(0, QCoreApplication::translate("MainWindow", "0", nullptr));
        activeAxisCombo->setItemText(1, QCoreApplication::translate("MainWindow", "1", nullptr));
        activeAxisCombo->setItemText(2, QCoreApplication::translate("MainWindow", "2", nullptr));
        activeAxisCombo->setItemText(3, QCoreApplication::translate("MainWindow", "3", nullptr));
        activeAxisCombo->setItemText(4, QCoreApplication::translate("MainWindow", "4", nullptr));
        activeAxisCombo->setItemText(5, QCoreApplication::translate("MainWindow", "5", nullptr));
        activeAxisCombo->setItemText(6, QCoreApplication::translate("MainWindow", "6", nullptr));
        activeAxisCombo->setItemText(7, QCoreApplication::translate("MainWindow", "7", nullptr));

        holdAxisLabel->setText(QCoreApplication::translate("MainWindow", "\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264", nullptr));
        holdAxisCombo->setItemText(0, QCoreApplication::translate("MainWindow", "0", nullptr));
        holdAxisCombo->setItemText(1, QCoreApplication::translate("MainWindow", "1", nullptr));
        holdAxisCombo->setItemText(2, QCoreApplication::translate("MainWindow", "2", nullptr));
        holdAxisCombo->setItemText(3, QCoreApplication::translate("MainWindow", "3", nullptr));
        holdAxisCombo->setItemText(4, QCoreApplication::translate("MainWindow", "4", nullptr));
        holdAxisCombo->setItemText(5, QCoreApplication::translate("MainWindow", "5", nullptr));
        holdAxisCombo->setItemText(6, QCoreApplication::translate("MainWindow", "6", nullptr));
        holdAxisCombo->setItemText(7, QCoreApplication::translate("MainWindow", "7", nullptr));

        activeDeltaLabel->setText(QCoreApplication::translate("MainWindow", "\344\270\273\345\212\250\350\275\264\344\275\215\347\247\273", nullptr));
        activeDeltaSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        holdDeltaLabel->setText(QCoreApplication::translate("MainWindow", "\347\254\254\344\272\214\350\275\264\344\275\215\347\247\273", nullptr));
        holdDeltaSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        durationLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\346\227\266\351\225\277", nullptr));
        durationSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        producerPeriodLabel->setText(QCoreApplication::translate("MainWindow", "\344\272\247\347\202\271\345\221\250\346\234\237", nullptr));
        producerPeriodSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        contiGroup->setTitle(QCoreApplication::translate("MainWindow", "\350\277\236\347\273\255\346\217\222\350\241\245\345\217\202\346\225\260", nullptr));
        maxVelocityLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\200\345\244\247\347\237\242\351\207\217\351\200\237\345\272\246", nullptr));
        maxVelocitySpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        accelerationLabel->setText(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\346\227\266\351\227\264", nullptr));
        accelerationSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        decelerationLabel->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\346\227\266\351\227\264", nullptr));
        decelerationSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        sCurveLabel->setText(QCoreApplication::translate("MainWindow", "S \346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
        sCurveSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        speedRatioLabel->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246\345\200\215\347\216\207", nullptr));
        lookaheadLabel->setText(QCoreApplication::translate("MainWindow", "\345\260\217\347\272\277\346\256\265\345\211\215\347\236\273", nullptr));
        lookaheadCheck->setText(QCoreApplication::translate("MainWindow", "\345\220\257\347\224\250", nullptr));
        pathErrorLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\345\205\201\350\256\270\350\257\257\345\267\256", nullptr));
        pathErrorSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        preloadSegmentsLabel->setText(QCoreApplication::translate("MainWindow", "\345\220\257\345\212\250\351\242\204\345\216\213\346\256\265\346\225\260", nullptr));
        targetBufferLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\256\346\240\207\347\274\223\345\206\262\346\256\265\346\225\260", nullptr));
        lowBufferLabel->setText(QCoreApplication::translate("MainWindow", "\344\275\216\346\260\264\344\275\215\346\256\265\346\225\260", nullptr));
        startButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\350\277\236\347\273\255\346\217\222\350\241\245", nullptr));
        stopButton->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\201\234\346\255\242", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(contiTestTab), QCoreApplication::translate("MainWindow", "\350\277\236\347\273\255\346\217\222\350\241\245\346\265\213\350\257\225", nullptr));
        baseConfigGroup->setTitle(QCoreApplication::translate("MainWindow", "E5000 \345\237\272\347\241\200\351\205\215\347\275\256", nullptr));
        axisRangeLabel->setText(QCoreApplication::translate("MainWindow", "\345\237\272\347\241\200\351\205\215\347\275\256\350\275\264\350\214\203\345\233\264", nullptr));
        axisRangeValueLabel->setText(QCoreApplication::translate("MainWindow", "\345\275\223\345\211\215\347\224\261\342\200\234\344\270\273\345\212\250\350\275\264\342\200\235\345\222\214\342\200\234\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264\342\200\235\351\200\211\346\213\251", nullptr));
        equivLabel->setText(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
        equivValueLabel->setText(QCoreApplication::translate("MainWindow", "500.622 pulse/deg\357\274\210180224 pulse/rev\357\274\211", nullptr));
        feedbackSourceLabel->setText(QCoreApplication::translate("MainWindow", "\345\217\215\351\246\210\346\235\245\346\272\220", nullptr));
        feedbackSourceValueLabel->setText(QCoreApplication::translate("MainWindow", "Trace \345\220\214\345\270\247\357\274\232\346\214\207\344\273\244\344\275\215\347\275\256(type 5) + \345\256\236\351\231\205\344\275\215\347\275\256(type 6)", nullptr));
        singleAxisJogGroup->setTitle(QCoreApplication::translate("MainWindow", "\345\215\225\347\224\265\346\234\272\347\202\271\345\212\250\357\274\210\347\233\270\345\257\271\344\275\215\347\247\273\357\274\211", nullptr));
        jogHintLabel->setText(QCoreApplication::translate("MainWindow", "\342\200\234\347\233\270\345\257\271\347\247\273\345\212\250\350\267\235\347\246\273\342\200\235\347\233\270\345\257\271\344\272\216\344\270\213\345\217\221\346\227\266\345\210\273\347\232\204\345\275\223\345\211\215\344\275\215\347\275\256\357\274\232\346\255\243\350\264\237\345\217\267\345\206\263\345\256\232\346\226\271\345\220\221\343\200\202\350\275\257\344\273\266\351\233\266\344\275\215\345\222\214\342\200\234\346\230\276\347\244\272\347\273\235\345\257\271\344\275\215\347\275\256\342\200\235\344\273\205\345\275\261\345\223\215\344\275\215\347\275\256\346\230\276\347\244\272\357\274\214\344\270\215\345\217\202\344\270\216\350\277\220\345\212\250\344\270\213\345\217\221\357\274\233\345\274\200\345\247\213\345\211\215\345\205\210\344\275\277\350\203\275\346\265\213\350\257\225\350\275\264\343\200\202\350\277\236\347\273\255\346\217\222\350\241\245\345\207\206\345\244\207\346\210\226\350\277\220\350\241\214\346\234\237\351\227\264\347\246\201\346\255\242\347\202\271\345\212\250\343\200\202", nullptr));
        jogAxisLabel->setText(QCoreApplication::translate("MainWindow", "\346\265\213\350\257\225\350\275\264", nullptr));
        jogAxisCombo->setItemText(0, QCoreApplication::translate("MainWindow", "0", nullptr));
        jogAxisCombo->setItemText(1, QCoreApplication::translate("MainWindow", "1", nullptr));
        jogAxisCombo->setItemText(2, QCoreApplication::translate("MainWindow", "2", nullptr));
        jogAxisCombo->setItemText(3, QCoreApplication::translate("MainWindow", "3", nullptr));
        jogAxisCombo->setItemText(4, QCoreApplication::translate("MainWindow", "4", nullptr));
        jogAxisCombo->setItemText(5, QCoreApplication::translate("MainWindow", "5", nullptr));
        jogAxisCombo->setItemText(6, QCoreApplication::translate("MainWindow", "6", nullptr));
        jogAxisCombo->setItemText(7, QCoreApplication::translate("MainWindow", "7", nullptr));

        jogActualPositionLabel->setText(QCoreApplication::translate("MainWindow", "\345\256\236\351\231\205\344\275\215\347\275\256\357\274\210Trace\357\274\211", nullptr));
        jogActualPositionSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        jogShowAbsoluteCheck->setText(QCoreApplication::translate("MainWindow", "\346\230\276\347\244\272\347\273\235\345\257\271\344\275\215\347\275\256", nullptr));
        jogTargetPositionLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\270\345\257\271\347\247\273\345\212\250\350\267\235\347\246\273", nullptr));
        jogTargetPositionSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        jogVelocityLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\256\346\240\207\351\200\237\345\272\246", nullptr));
        jogVelocitySpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        jogEnableButton->setText(QCoreApplication::translate("MainWindow", "\344\275\277\350\203\275\346\265\213\350\257\225\350\275\264", nullptr));
        jogDisableButton->setText(QCoreApplication::translate("MainWindow", "\345\244\261\350\203\275\346\265\213\350\257\225\350\275\264", nullptr));
        jogUseActualPositionButton->setText(QCoreApplication::translate("MainWindow", "\347\233\270\345\257\271\344\275\215\347\247\273\347\275\256\351\233\266", nullptr));
#if QT_CONFIG(tooltip)
        jogSetCurrentZeroButton->setToolTip(QCoreApplication::translate("MainWindow", "\344\273\205\350\256\260\345\275\225\345\275\223\345\211\215\346\265\213\350\257\225\350\275\264\347\232\204 Trace \345\256\236\351\231\205\344\275\215\347\275\256\344\270\272\350\275\257\344\273\266\351\233\266\347\202\271\357\274\214\344\270\215\345\206\231\345\205\245\346\216\247\345\210\266\345\215\241\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        jogSetCurrentZeroButton->setText(QCoreApplication::translate("MainWindow", "\344\273\245\345\275\223\345\211\215\344\275\215\347\275\256\344\270\272\351\233\266\344\275\215", nullptr));
        jogStartButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\347\202\271\344\275\215\350\277\220\345\212\250", nullptr));
        jogStopButton->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\201\234\346\255\242", nullptr));
        jogEmergencyStopButton->setText(QCoreApplication::translate("MainWindow", "\347\253\213\345\215\263\345\201\234\346\255\242", nullptr));
        refreshFeedbackButton->setText(QCoreApplication::translate("MainWindow", "\347\253\213\345\215\263\345\210\267\346\226\260\345\217\215\351\246\210", nullptr));
        feedbackSummaryValueLabel->setText(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241\346\234\252\345\210\235\345\247\213\345\214\226", nullptr));
        QTableWidgetItem *___qtablewidgetitem = axisFeedbackTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("MainWindow", "\350\275\264\345\217\267", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = axisFeedbackTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("MainWindow", "\350\257\273\345\217\226\347\212\266\346\200\201", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = axisFeedbackTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("MainWindow", "\347\212\266\346\200\201\346\234\272", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = axisFeedbackTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("MainWindow", "\350\275\264\351\224\231\350\257\257\347\240\201", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = axisFeedbackTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("MainWindow", "\346\214\207\344\273\244\344\275\215\347\275\256 (\302\260)", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = axisFeedbackTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("MainWindow", "\345\256\236\351\231\205\344\275\215\347\275\256 (\302\260)", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = axisFeedbackTable->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("MainWindow", "\350\267\237\351\232\217\350\257\257\345\267\256 (\302\260)", nullptr));
        QTableWidgetItem *___qtablewidgetitem7 = axisFeedbackTable->horizontalHeaderItem(7);
        ___qtablewidgetitem7->setText(QCoreApplication::translate("MainWindow", "\345\244\207\346\263\250", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(axisFeedbackTab), QCoreApplication::translate("MainWindow", "\350\275\264\351\205\215\347\275\256\344\270\216\345\217\215\351\246\210", nullptr));
        telemetryRecordGroup->setTitle(QCoreApplication::translate("MainWindow", "\351\230\266\346\256\265 A\357\274\232\344\270\244\350\275\264 Trace \345\274\202\346\255\245\350\256\260\345\275\225", nullptr));
        startRecordingButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\350\256\260\345\275\225", nullptr));
        stopRecordingButton->setText(QCoreApplication::translate("MainWindow", "\345\201\234\346\255\242\350\256\260\345\275\225\345\271\266\345\206\231\345\205\245\346\226\207\344\273\266", nullptr));
        recordingStateValueLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\252\350\256\260\345\275\225", nullptr));
        recordPathLabel->setText(QCoreApplication::translate("MainWindow", "\350\276\223\345\207\272\347\233\256\345\275\225", nullptr));
        recordPathValueLabel->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\350\256\260\345\275\225\345\220\216\350\207\252\345\212\250\345\210\233\345\273\272 records/run_*", nullptr));
        recordStatsLabel->setText(QCoreApplication::translate("MainWindow", "\345\206\231\345\205\245 / \351\230\237\345\210\227 / \344\270\242\345\270\247", nullptr));
        recordStatsValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0 / 0", nullptr));
        telemetryHintLabel->setText(QCoreApplication::translate("MainWindow", "\346\233\262\347\272\277\344\273\205\344\273\245 20 Hz \346\230\276\347\244\272\346\234\200\346\226\260 Trace \345\277\253\347\205\247\357\274\233\345\216\237\345\247\213 500 us Trace \345\270\247\347\224\261\347\213\254\347\253\213\345\206\231\347\233\230\347\272\277\347\250\213\344\277\235\345\255\230\357\274\214\344\270\215\345\234\250 UI \347\272\277\347\250\213\347\273\230\345\210\266\346\210\226\345\206\231\345\205\245\346\226\207\346\234\254\343\200\202", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(telemetryTab), QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\350\256\260\345\275\225\344\270\216\346\233\262\347\272\277", nullptr));
        extensionHintLabel->setText(QCoreApplication::translate("MainWindow", "\346\255\244\351\241\265\351\235\242\351\242\204\347\225\231\347\273\231\345\220\216\347\273\255\347\232\204 8 \350\275\264 CDPR\343\200\201\345\212\250\345\212\233\345\255\246\346\261\202\350\247\243\343\200\201\345\212\233\344\274\240\346\204\237\345\231\250\345\222\214\347\274\223\345\206\262\346\260\264\344\275\215\350\207\252\345\212\250\350\260\203\351\200\237\347\255\211\345\212\237\350\203\275\343\200\202", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(extensionTab), QCoreApplication::translate("MainWindow", "\346\211\251\345\261\225\345\212\237\350\203\275\357\274\210\351\242\204\347\225\231\357\274\211", nullptr));
        logGroup->setTitle(QCoreApplication::translate("MainWindow", "\350\277\220\350\241\214\346\227\245\345\277\227", nullptr));
        logHintLabel->setText(QCoreApplication::translate("MainWindow", "\346\211\200\346\234\211\351\241\265\351\235\242\345\205\261\347\224\250", nullptr));
        clearLogButton->setText(QCoreApplication::translate("MainWindow", "\346\270\205\351\231\244\346\227\245\345\277\227", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
