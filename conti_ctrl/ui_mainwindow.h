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
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "widgets/ZoomableChartView.h"

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
    QLabel *stateLabel;
    QLabel *stateValueLabel;
    QLabel *busCycleSettingLabel;
    QComboBox *busCycleCombo;
    QLabel *busCycleReadLabel;
    QWidget *busCycleReadContainer;
    QHBoxLayout *busCycleReadLayout;
    QLabel *busCycleReadValueLabel;
    QPushButton *readBusCycleButton;
    QLabel *traceSampleLabel;
    QLabel *traceSampleValueLabel;
    QGroupBox *globalActionGroup;
    QVBoxLayout *globalActionLayout;
    QPushButton *initializeButton;
    QPushButton *closeBoardButton;
    QGroupBox *axisEnableStatusGroup;
    QGridLayout *axisEnableStatusLayout;
    QLabel *axis0EnableStateLabel;
    QLabel *axis1EnableStateLabel;
    QLabel *axis2EnableStateLabel;
    QLabel *axis3EnableStateLabel;
    QLabel *axis4EnableStateLabel;
    QLabel *axis5EnableStateLabel;
    QLabel *axis6EnableStateLabel;
    QLabel *axis7EnableStateLabel;
    QPushButton *enableAllAxesButton;
    QPushButton *disableAllAxesButton;
    QGroupBox *globalEmergencyGroup;
    QVBoxLayout *globalEmergencyLayout;
    QPushButton *emergencyStopButton;
    QLabel *globalEmergencyHintLabel;
    QSpacerItem *globalControlSpacer;
    QSplitter *mainSplitter;
    QTabWidget *tabWidget;
    QWidget *contiTestTab;
    QVBoxLayout *contiTestOuterLayout;
    QScrollArea *contiTestScrollArea;
    QWidget *contiTestScrollContent;
    QVBoxLayout *contiTestLayout;
    QHBoxLayout *settingsLayout;
    QWidget *contiLeftPanel;
    QVBoxLayout *contiLeftPanelLayout;
    QGroupBox *contiRuntimeStatusGroup;
    QGridLayout *contiRuntimeStatusLayout;
    QPushButton *contiEnableAxesButton;
    QPushButton *contiDisableAxesButton;
    QLabel *contiRunStateLabel;
    QLabel *contiRunStateValueLabel;
    QLabel *contiMarkLabel;
    QLabel *contiMarkValueLabel;
    QLabel *contiBufferLabel;
    QLabel *contiBufferValueLabel;
    QLabel *contiSpaceLabel;
    QLabel *contiSpaceValueLabel;
    QLabel *contiHostQueueLabel;
    QLabel *contiHostQueueValueLabel;
    QLabel *contiPlanningAlignmentLabel;
    QLabel *contiPlanningAlignmentValueLabel;
    QLabel *contiExpectedPlanTimeLabel;
    QLabel *contiExpectedPlanTimeValueLabel;
    QLabel *contiPhaseErrorLabel;
    QLabel *contiPhaseErrorValueLabel;
    QLabel *contiBufferTimeLabel;
    QLabel *contiBufferTimeValueLabel;
    QLabel *contiRatioApiAgeLabel;
    QLabel *contiRatioApiAgeValueLabel;
    QLabel *contiRatioDiagLabel;
    QLabel *contiRatioDiagValueLabel;
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
    QLabel *cardUnitDefinitionLabel;
    QComboBox *cardUnitDefinitionCombo;
    QLabel *trajectoryPointModeLabel;
    QComboBox *trajectoryPointModeCombo;
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
    QWidget *preloadModeContainer;
    QVBoxLayout *preloadModeLayout;
    QSpinBox *preloadSegmentsSpin;
    QCheckBox *preloadAllTrajectoryCheck;
    QLabel *targetBufferLabel;
    QSpinBox *targetBufferSpin;
    QLabel *lowBufferLabel;
    QSpinBox *lowBufferSpin;
    QLabel *timeSyncEnableLabel;
    QCheckBox *timeSyncEnableCheck;
    QLabel *criticalBufferLabel;
    QSpinBox *criticalBufferSpin;
    QLabel *ratioMinLabel;
    QDoubleSpinBox *ratioMinSpin;
    QLabel *phaseGainLabel;
    QDoubleSpinBox *phaseGainSpin;
    QLabel *ratioDeadbandLabel;
    QDoubleSpinBox *ratioDeadbandSpin;
    QLabel *ratioStepLabel;
    QDoubleSpinBox *ratioStepSpin;
    QLabel *ratioPeriodLabel;
    QSpinBox *ratioPeriodSpin;
    QLabel *markOffsetLabel;
    QSpinBox *markOffsetSpin;
    QLabel *bufferGainLabel;
    QDoubleSpinBox *bufferGainSpin;
    QLabel *executionDelayLabel;
    QSpinBox *executionDelaySpin;
    QLabel *phaseDeadbandLabel;
    QSpinBox *phaseDeadbandSpin;
    QLabel *ratioApiIntervalLabel;
    QSpinBox *ratioApiIntervalSpin;
    QLabel *ratioSafetyApiIntervalLabel;
    QSpinBox *ratioSafetyApiIntervalSpin;
    QHBoxLayout *contiButtonLayout;
    QPushButton *startButton;
    QPushButton *stopButton;
    QGroupBox *contiTrajectoryGroup;
    QVBoxLayout *contiTrajectoryLayout;
    ZoomableChartView *contiTrajectoryChartView;
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
    ZoomableChartView *positionChartView;
    ZoomableChartView *followingErrorChartView;
    QLabel *telemetryHintLabel;
    QWidget *velocityControlTab;
    QVBoxLayout *velocityControlTabLayout;
    QScrollArea *velocityControlScrollArea;
    QWidget *velocityControlScrollContent;
    QVBoxLayout *velocityControlContentLayout;
    QGroupBox *velocityControlParameterGroup;
    QGridLayout *velocityControlParameterLayout;
    QGroupBox *velocityTrajectoryGroup;
    QGridLayout *velocityTrajectoryLayout;
    QLabel *velocityAxisLabel;
    QComboBox *velocityAxisCombo;
    QLabel *velocityDeltaLabel;
    QDoubleSpinBox *velocityDeltaSpin;
    QLabel *velocityDurationLabel;
    QDoubleSpinBox *velocityDurationSpin;
    QLabel *velocityControlPeriodLabel;
    QSpinBox *velocityControlPeriodSpin;
    QSpacerItem *velocityTrajectorySpacer;
    QGroupBox *velocityClosedLoopGroup;
    QGridLayout *velocityClosedLoopLayout;
    QCheckBox *velocityFeedforwardCheck;
    QDoubleSpinBox *velocityFeedforwardGainSpin;
    QCheckBox *velocityPidEnableCheck;
    QLabel *velocityKpLabel;
    QDoubleSpinBox *velocityKpSpin;
    QLabel *velocityKiLabel;
    QDoubleSpinBox *velocityKiSpin;
    QLabel *velocityKdLabel;
    QDoubleSpinBox *velocityKdSpin;
    QLabel *velocityIntegralLimitLabel;
    QDoubleSpinBox *velocityIntegralLimitSpin;
    QLabel *velocityMaxCorrectionLabel;
    QDoubleSpinBox *velocityMaxCorrectionSpin;
    QSpacerItem *velocityClosedLoopSpacer;
    QGroupBox *velocityLimitGroup;
    QGridLayout *velocityLimitLayout;
    QLabel *velocityMaxSpeedLabel;
    QDoubleSpinBox *velocityMaxSpeedSpin;
    QLabel *velocityMaxAccelerationLabel;
    QDoubleSpinBox *velocityMaxAccelerationSpin;
    QLabel *velocityChangeTimeLabel;
    QDoubleSpinBox *velocityChangeTimeSpin;
    QLabel *velocityStartThresholdLabel;
    QDoubleSpinBox *velocityStartThresholdSpin;
    QSpacerItem *velocityLimitSpacer;
    QGroupBox *velocityCriterionGroup;
    QGridLayout *velocityCriterionLayout;
    QLabel *velocityPositionToleranceLabel;
    QDoubleSpinBox *velocityPositionToleranceSpin;
    QLabel *velocitySpeedToleranceLabel;
    QDoubleSpinBox *velocitySpeedToleranceSpin;
    QLabel *velocityStableDwellLabel;
    QSpinBox *velocityStableDwellSpin;
    QLabel *velocityFinishTimeoutLabel;
    QSpinBox *velocityFinishTimeoutSpin;
    QLabel *velocityMaxFollowingErrorLabel;
    QDoubleSpinBox *velocityMaxFollowingErrorSpin;
    QLabel *velocityTraceTimeoutLabel;
    QSpinBox *velocityTraceTimeoutSpin;
    QSpacerItem *velocityCriterionSpacer;
    QGroupBox *velocityControlOperationGroup;
    QGridLayout *velocityControlOperationLayout;
    QPushButton *velocityEnableAxisButton;
    QPushButton *velocityDisableAxisButton;
    QPushButton *velocityResetButton;
    QPushButton *velocityStartButton;
    QPushButton *velocityStopButton;
    QPushButton *velocityEmergencyStopButton;
    QPushButton *velocityClearCurvesButton;
    QLabel *velocityStateLabel;
    QLabel *velocityStateValueLabel;
    QLabel *velocityTimingLabel;
    QLabel *velocityTimingValueLabel;
    QLabel *velocityPositionStatusLabel;
    QLabel *velocityPositionStatusValueLabel;
    QLabel *velocitySpeedStatusLabel;
    QLabel *velocitySpeedStatusValueLabel;
    QLabel *velocityPidStatusLabel;
    QLabel *velocityPidStatusValueLabel;
    QSplitter *velocityControlChartSplitter;
    ZoomableChartView *velocityPositionChartView;
    ZoomableChartView *velocityErrorChartView;
    ZoomableChartView *velocitySpeedChartView;
    QLabel *velocityControlHintLabel;
    QWidget *extensionTab;
    QVBoxLayout *extensionLayout;
    QLabel *extensionHintLabel;
    QSpacerItem *extensionVerticalSpacer;
    QGroupBox *logGroup;
    QVBoxLayout *logLayout;
    QHBoxLayout *logToolbarLayout;
    QPushButton *copyLogButton;
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
        stateLabel = new QLabel(globalStatusGroup);
        stateLabel->setObjectName("stateLabel");

        statusForm->setWidget(1, QFormLayout::LabelRole, stateLabel);

        stateValueLabel = new QLabel(globalStatusGroup);
        stateValueLabel->setObjectName("stateValueLabel");
        stateValueLabel->setWordWrap(true);

        statusForm->setWidget(1, QFormLayout::FieldRole, stateValueLabel);

        busCycleSettingLabel = new QLabel(globalStatusGroup);
        busCycleSettingLabel->setObjectName("busCycleSettingLabel");
        busCycleSettingLabel->setWordWrap(true);

        statusForm->setWidget(7, QFormLayout::LabelRole, busCycleSettingLabel);

        busCycleCombo = new QComboBox(globalStatusGroup);
        busCycleCombo->addItem(QString());
        busCycleCombo->addItem(QString());
        busCycleCombo->addItem(QString());
        busCycleCombo->addItem(QString());
        busCycleCombo->setObjectName("busCycleCombo");

        statusForm->setWidget(7, QFormLayout::FieldRole, busCycleCombo);

        busCycleReadLabel = new QLabel(globalStatusGroup);
        busCycleReadLabel->setObjectName("busCycleReadLabel");
        busCycleReadLabel->setWordWrap(true);

        statusForm->setWidget(8, QFormLayout::LabelRole, busCycleReadLabel);

        busCycleReadContainer = new QWidget(globalStatusGroup);
        busCycleReadContainer->setObjectName("busCycleReadContainer");
        busCycleReadLayout = new QHBoxLayout(busCycleReadContainer);
        busCycleReadLayout->setObjectName("busCycleReadLayout");
        busCycleReadLayout->setContentsMargins(0, 0, 0, 0);
        busCycleReadValueLabel = new QLabel(busCycleReadContainer);
        busCycleReadValueLabel->setObjectName("busCycleReadValueLabel");

        busCycleReadLayout->addWidget(busCycleReadValueLabel);

        readBusCycleButton = new QPushButton(busCycleReadContainer);
        readBusCycleButton->setObjectName("readBusCycleButton");
        readBusCycleButton->setEnabled(false);

        busCycleReadLayout->addWidget(readBusCycleButton);


        statusForm->setWidget(8, QFormLayout::FieldRole, busCycleReadContainer);

        traceSampleLabel = new QLabel(globalStatusGroup);
        traceSampleLabel->setObjectName("traceSampleLabel");
        traceSampleLabel->setWordWrap(true);

        statusForm->setWidget(9, QFormLayout::LabelRole, traceSampleLabel);

        traceSampleValueLabel = new QLabel(globalStatusGroup);
        traceSampleValueLabel->setObjectName("traceSampleValueLabel");
        traceSampleValueLabel->setWordWrap(true);

        statusForm->setWidget(9, QFormLayout::FieldRole, traceSampleValueLabel);


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

        axisEnableStatusGroup = new QGroupBox(globalActionGroup);
        axisEnableStatusGroup->setObjectName("axisEnableStatusGroup");
        axisEnableStatusGroup->setStyleSheet(QString::fromUtf8("QLabel { color: white; background-color: #757575; border-radius: 4px; padding: 4px; }"));
        axisEnableStatusLayout = new QGridLayout(axisEnableStatusGroup);
        axisEnableStatusLayout->setObjectName("axisEnableStatusLayout");
        axis0EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis0EnableStateLabel->setObjectName("axis0EnableStateLabel");
        axis0EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis0EnableStateLabel, 0, 0, 1, 1);

        axis1EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis1EnableStateLabel->setObjectName("axis1EnableStateLabel");
        axis1EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis1EnableStateLabel, 0, 1, 1, 1);

        axis2EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis2EnableStateLabel->setObjectName("axis2EnableStateLabel");
        axis2EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis2EnableStateLabel, 1, 0, 1, 1);

        axis3EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis3EnableStateLabel->setObjectName("axis3EnableStateLabel");
        axis3EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis3EnableStateLabel, 1, 1, 1, 1);

        axis4EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis4EnableStateLabel->setObjectName("axis4EnableStateLabel");
        axis4EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis4EnableStateLabel, 2, 0, 1, 1);

        axis5EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis5EnableStateLabel->setObjectName("axis5EnableStateLabel");
        axis5EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis5EnableStateLabel, 2, 1, 1, 1);

        axis6EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis6EnableStateLabel->setObjectName("axis6EnableStateLabel");
        axis6EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis6EnableStateLabel, 3, 0, 1, 1);

        axis7EnableStateLabel = new QLabel(axisEnableStatusGroup);
        axis7EnableStateLabel->setObjectName("axis7EnableStateLabel");
        axis7EnableStateLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        axisEnableStatusLayout->addWidget(axis7EnableStateLabel, 3, 1, 1, 1);

        enableAllAxesButton = new QPushButton(axisEnableStatusGroup);
        enableAllAxesButton->setObjectName("enableAllAxesButton");
        enableAllAxesButton->setEnabled(false);

        axisEnableStatusLayout->addWidget(enableAllAxesButton, 4, 0, 1, 1);

        disableAllAxesButton = new QPushButton(axisEnableStatusGroup);
        disableAllAxesButton->setObjectName("disableAllAxesButton");
        disableAllAxesButton->setEnabled(false);

        axisEnableStatusLayout->addWidget(disableAllAxesButton, 4, 1, 1, 1);


        globalActionLayout->addWidget(axisEnableStatusGroup);


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
        contiTestOuterLayout = new QVBoxLayout(contiTestTab);
        contiTestOuterLayout->setObjectName("contiTestOuterLayout");
        contiTestScrollArea = new QScrollArea(contiTestTab);
        contiTestScrollArea->setObjectName("contiTestScrollArea");
        contiTestScrollArea->setWidgetResizable(true);
        contiTestScrollContent = new QWidget();
        contiTestScrollContent->setObjectName("contiTestScrollContent");
        contiTestScrollContent->setGeometry(QRect(0, 0, 781, 1063));
        contiTestLayout = new QVBoxLayout(contiTestScrollContent);
        contiTestLayout->setObjectName("contiTestLayout");
        settingsLayout = new QHBoxLayout();
        settingsLayout->setObjectName("settingsLayout");
        contiLeftPanel = new QWidget(contiTestScrollContent);
        contiLeftPanel->setObjectName("contiLeftPanel");
        contiLeftPanelLayout = new QVBoxLayout(contiLeftPanel);
        contiLeftPanelLayout->setObjectName("contiLeftPanelLayout");
        contiLeftPanelLayout->setContentsMargins(0, 0, 0, 0);
        contiRuntimeStatusGroup = new QGroupBox(contiLeftPanel);
        contiRuntimeStatusGroup->setObjectName("contiRuntimeStatusGroup");
        contiRuntimeStatusLayout = new QGridLayout(contiRuntimeStatusGroup);
        contiRuntimeStatusLayout->setObjectName("contiRuntimeStatusLayout");
        contiEnableAxesButton = new QPushButton(contiRuntimeStatusGroup);
        contiEnableAxesButton->setObjectName("contiEnableAxesButton");

        contiRuntimeStatusLayout->addWidget(contiEnableAxesButton, 0, 0, 1, 2);

        contiDisableAxesButton = new QPushButton(contiRuntimeStatusGroup);
        contiDisableAxesButton->setObjectName("contiDisableAxesButton");

        contiRuntimeStatusLayout->addWidget(contiDisableAxesButton, 0, 2, 1, 2);

        contiRunStateLabel = new QLabel(contiRuntimeStatusGroup);
        contiRunStateLabel->setObjectName("contiRunStateLabel");

        contiRuntimeStatusLayout->addWidget(contiRunStateLabel, 1, 0, 1, 1);

        contiRunStateValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiRunStateValueLabel->setObjectName("contiRunStateValueLabel");

        contiRuntimeStatusLayout->addWidget(contiRunStateValueLabel, 1, 1, 1, 1);

        contiMarkLabel = new QLabel(contiRuntimeStatusGroup);
        contiMarkLabel->setObjectName("contiMarkLabel");

        contiRuntimeStatusLayout->addWidget(contiMarkLabel, 1, 2, 1, 1);

        contiMarkValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiMarkValueLabel->setObjectName("contiMarkValueLabel");

        contiRuntimeStatusLayout->addWidget(contiMarkValueLabel, 1, 3, 1, 1);

        contiBufferLabel = new QLabel(contiRuntimeStatusGroup);
        contiBufferLabel->setObjectName("contiBufferLabel");

        contiRuntimeStatusLayout->addWidget(contiBufferLabel, 2, 0, 1, 1);

        contiBufferValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiBufferValueLabel->setObjectName("contiBufferValueLabel");

        contiRuntimeStatusLayout->addWidget(contiBufferValueLabel, 2, 1, 1, 1);

        contiSpaceLabel = new QLabel(contiRuntimeStatusGroup);
        contiSpaceLabel->setObjectName("contiSpaceLabel");

        contiRuntimeStatusLayout->addWidget(contiSpaceLabel, 2, 2, 1, 1);

        contiSpaceValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiSpaceValueLabel->setObjectName("contiSpaceValueLabel");

        contiRuntimeStatusLayout->addWidget(contiSpaceValueLabel, 2, 3, 1, 1);

        contiHostQueueLabel = new QLabel(contiRuntimeStatusGroup);
        contiHostQueueLabel->setObjectName("contiHostQueueLabel");

        contiRuntimeStatusLayout->addWidget(contiHostQueueLabel, 3, 0, 1, 1);

        contiHostQueueValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiHostQueueValueLabel->setObjectName("contiHostQueueValueLabel");

        contiRuntimeStatusLayout->addWidget(contiHostQueueValueLabel, 3, 1, 1, 1);

        contiPlanningAlignmentLabel = new QLabel(contiRuntimeStatusGroup);
        contiPlanningAlignmentLabel->setObjectName("contiPlanningAlignmentLabel");

        contiRuntimeStatusLayout->addWidget(contiPlanningAlignmentLabel, 3, 2, 1, 1);

        contiPlanningAlignmentValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiPlanningAlignmentValueLabel->setObjectName("contiPlanningAlignmentValueLabel");
        contiPlanningAlignmentValueLabel->setWordWrap(true);

        contiRuntimeStatusLayout->addWidget(contiPlanningAlignmentValueLabel, 3, 3, 1, 1);

        contiExpectedPlanTimeLabel = new QLabel(contiRuntimeStatusGroup);
        contiExpectedPlanTimeLabel->setObjectName("contiExpectedPlanTimeLabel");

        contiRuntimeStatusLayout->addWidget(contiExpectedPlanTimeLabel, 4, 0, 1, 1);

        contiExpectedPlanTimeValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiExpectedPlanTimeValueLabel->setObjectName("contiExpectedPlanTimeValueLabel");

        contiRuntimeStatusLayout->addWidget(contiExpectedPlanTimeValueLabel, 4, 1, 1, 1);

        contiPhaseErrorLabel = new QLabel(contiRuntimeStatusGroup);
        contiPhaseErrorLabel->setObjectName("contiPhaseErrorLabel");

        contiRuntimeStatusLayout->addWidget(contiPhaseErrorLabel, 4, 2, 1, 1);

        contiPhaseErrorValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiPhaseErrorValueLabel->setObjectName("contiPhaseErrorValueLabel");

        contiRuntimeStatusLayout->addWidget(contiPhaseErrorValueLabel, 4, 3, 1, 1);

        contiBufferTimeLabel = new QLabel(contiRuntimeStatusGroup);
        contiBufferTimeLabel->setObjectName("contiBufferTimeLabel");

        contiRuntimeStatusLayout->addWidget(contiBufferTimeLabel, 5, 0, 1, 1);

        contiBufferTimeValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiBufferTimeValueLabel->setObjectName("contiBufferTimeValueLabel");

        contiRuntimeStatusLayout->addWidget(contiBufferTimeValueLabel, 5, 1, 1, 1);

        contiRatioApiAgeLabel = new QLabel(contiRuntimeStatusGroup);
        contiRatioApiAgeLabel->setObjectName("contiRatioApiAgeLabel");

        contiRuntimeStatusLayout->addWidget(contiRatioApiAgeLabel, 5, 2, 1, 1);

        contiRatioApiAgeValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiRatioApiAgeValueLabel->setObjectName("contiRatioApiAgeValueLabel");

        contiRuntimeStatusLayout->addWidget(contiRatioApiAgeValueLabel, 5, 3, 1, 1);

        contiRatioDiagLabel = new QLabel(contiRuntimeStatusGroup);
        contiRatioDiagLabel->setObjectName("contiRatioDiagLabel");
        contiRatioDiagLabel->setWordWrap(true);

        contiRuntimeStatusLayout->addWidget(contiRatioDiagLabel, 6, 0, 1, 1);

        contiRatioDiagValueLabel = new QLabel(contiRuntimeStatusGroup);
        contiRatioDiagValueLabel->setObjectName("contiRatioDiagValueLabel");
        contiRatioDiagValueLabel->setWordWrap(true);

        contiRuntimeStatusLayout->addWidget(contiRatioDiagValueLabel, 6, 1, 1, 3);


        contiLeftPanelLayout->addWidget(contiRuntimeStatusGroup);

        testGroup = new QGroupBox(contiLeftPanel);
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

        cardUnitDefinitionLabel = new QLabel(testGroup);
        cardUnitDefinitionLabel->setObjectName("cardUnitDefinitionLabel");

        testForm->setWidget(10, QFormLayout::LabelRole, cardUnitDefinitionLabel);

        cardUnitDefinitionCombo = new QComboBox(testGroup);
        cardUnitDefinitionCombo->addItem(QString());
        cardUnitDefinitionCombo->addItem(QString());
        cardUnitDefinitionCombo->addItem(QString());
        cardUnitDefinitionCombo->setObjectName("cardUnitDefinitionCombo");

        testForm->setWidget(10, QFormLayout::FieldRole, cardUnitDefinitionCombo);

        trajectoryPointModeLabel = new QLabel(testGroup);
        trajectoryPointModeLabel->setObjectName("trajectoryPointModeLabel");

        testForm->setWidget(11, QFormLayout::LabelRole, trajectoryPointModeLabel);

        trajectoryPointModeCombo = new QComboBox(testGroup);
        trajectoryPointModeCombo->addItem(QString());
        trajectoryPointModeCombo->addItem(QString());
        trajectoryPointModeCombo->setObjectName("trajectoryPointModeCombo");

        testForm->setWidget(11, QFormLayout::FieldRole, trajectoryPointModeCombo);


        contiLeftPanelLayout->addWidget(testGroup);


        settingsLayout->addWidget(contiLeftPanel);

        contiGroup = new QGroupBox(contiTestScrollContent);
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
        speedRatioSpin->setMinimum(0.200000000000000);
        speedRatioSpin->setMaximum(2.000000000000000);
        speedRatioSpin->setSingleStep(0.050000000000000);
        speedRatioSpin->setValue(1.000000000000000);

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

        preloadModeContainer = new QWidget(contiGroup);
        preloadModeContainer->setObjectName("preloadModeContainer");
        preloadModeLayout = new QVBoxLayout(preloadModeContainer);
        preloadModeLayout->setObjectName("preloadModeLayout");
        preloadModeLayout->setContentsMargins(0, 0, 0, 0);
        preloadSegmentsSpin = new QSpinBox(preloadModeContainer);
        preloadSegmentsSpin->setObjectName("preloadSegmentsSpin");
        preloadSegmentsSpin->setMinimum(10);
        preloadSegmentsSpin->setMaximum(5000);
        preloadSegmentsSpin->setValue(200);

        preloadModeLayout->addWidget(preloadSegmentsSpin);

        preloadAllTrajectoryCheck = new QCheckBox(preloadModeContainer);
        preloadAllTrajectoryCheck->setObjectName("preloadAllTrajectoryCheck");

        preloadModeLayout->addWidget(preloadAllTrajectoryCheck);


        contiForm->setWidget(7, QFormLayout::FieldRole, preloadModeContainer);

        targetBufferLabel = new QLabel(contiGroup);
        targetBufferLabel->setObjectName("targetBufferLabel");

        contiForm->setWidget(8, QFormLayout::LabelRole, targetBufferLabel);

        targetBufferSpin = new QSpinBox(contiGroup);
        targetBufferSpin->setObjectName("targetBufferSpin");
        targetBufferSpin->setMinimum(10);
        targetBufferSpin->setMaximum(5000);
        targetBufferSpin->setValue(200);

        contiForm->setWidget(8, QFormLayout::FieldRole, targetBufferSpin);

        lowBufferLabel = new QLabel(contiGroup);
        lowBufferLabel->setObjectName("lowBufferLabel");

        contiForm->setWidget(9, QFormLayout::LabelRole, lowBufferLabel);

        lowBufferSpin = new QSpinBox(contiGroup);
        lowBufferSpin->setObjectName("lowBufferSpin");
        lowBufferSpin->setMinimum(1);
        lowBufferSpin->setMaximum(999);
        lowBufferSpin->setValue(100);

        contiForm->setWidget(9, QFormLayout::FieldRole, lowBufferSpin);

        timeSyncEnableLabel = new QLabel(contiGroup);
        timeSyncEnableLabel->setObjectName("timeSyncEnableLabel");

        contiForm->setWidget(10, QFormLayout::LabelRole, timeSyncEnableLabel);

        timeSyncEnableCheck = new QCheckBox(contiGroup);
        timeSyncEnableCheck->setObjectName("timeSyncEnableCheck");
        timeSyncEnableCheck->setChecked(true);

        contiForm->setWidget(10, QFormLayout::FieldRole, timeSyncEnableCheck);

        criticalBufferLabel = new QLabel(contiGroup);
        criticalBufferLabel->setObjectName("criticalBufferLabel");

        contiForm->setWidget(11, QFormLayout::LabelRole, criticalBufferLabel);

        criticalBufferSpin = new QSpinBox(contiGroup);
        criticalBufferSpin->setObjectName("criticalBufferSpin");
        criticalBufferSpin->setMinimum(1);
        criticalBufferSpin->setMaximum(4999);
        criticalBufferSpin->setValue(50);

        contiForm->setWidget(11, QFormLayout::FieldRole, criticalBufferSpin);

        ratioMinLabel = new QLabel(contiGroup);
        ratioMinLabel->setObjectName("ratioMinLabel");

        contiForm->setWidget(12, QFormLayout::LabelRole, ratioMinLabel);

        ratioMinSpin = new QDoubleSpinBox(contiGroup);
        ratioMinSpin->setObjectName("ratioMinSpin");
        ratioMinSpin->setDecimals(2);
        ratioMinSpin->setMinimum(0.010000000000000);
        ratioMinSpin->setMaximum(1.000000000000000);
        ratioMinSpin->setValue(0.200000000000000);

        contiForm->setWidget(12, QFormLayout::FieldRole, ratioMinSpin);

        phaseGainLabel = new QLabel(contiGroup);
        phaseGainLabel->setObjectName("phaseGainLabel");

        contiForm->setWidget(13, QFormLayout::LabelRole, phaseGainLabel);

        phaseGainSpin = new QDoubleSpinBox(contiGroup);
        phaseGainSpin->setObjectName("phaseGainSpin");
        phaseGainSpin->setDecimals(3);
        phaseGainSpin->setMaximum(10.000000000000000);
        phaseGainSpin->setSingleStep(0.100000000000000);
        phaseGainSpin->setValue(0.800000000000000);

        contiForm->setWidget(13, QFormLayout::FieldRole, phaseGainSpin);

        ratioDeadbandLabel = new QLabel(contiGroup);
        ratioDeadbandLabel->setObjectName("ratioDeadbandLabel");

        contiForm->setWidget(14, QFormLayout::LabelRole, ratioDeadbandLabel);

        ratioDeadbandSpin = new QDoubleSpinBox(contiGroup);
        ratioDeadbandSpin->setObjectName("ratioDeadbandSpin");
        ratioDeadbandSpin->setDecimals(3);
        ratioDeadbandSpin->setMaximum(1.000000000000000);
        ratioDeadbandSpin->setSingleStep(0.005000000000000);
        ratioDeadbandSpin->setValue(0.010000000000000);

        contiForm->setWidget(14, QFormLayout::FieldRole, ratioDeadbandSpin);

        ratioStepLabel = new QLabel(contiGroup);
        ratioStepLabel->setObjectName("ratioStepLabel");

        contiForm->setWidget(15, QFormLayout::LabelRole, ratioStepLabel);

        ratioStepSpin = new QDoubleSpinBox(contiGroup);
        ratioStepSpin->setObjectName("ratioStepSpin");
        ratioStepSpin->setDecimals(3);
        ratioStepSpin->setMinimum(0.001000000000000);
        ratioStepSpin->setMaximum(1.000000000000000);
        ratioStepSpin->setValue(0.020000000000000);

        contiForm->setWidget(15, QFormLayout::FieldRole, ratioStepSpin);

        ratioPeriodLabel = new QLabel(contiGroup);
        ratioPeriodLabel->setObjectName("ratioPeriodLabel");

        contiForm->setWidget(16, QFormLayout::LabelRole, ratioPeriodLabel);

        ratioPeriodSpin = new QSpinBox(contiGroup);
        ratioPeriodSpin->setObjectName("ratioPeriodSpin");
        ratioPeriodSpin->setMinimum(5);
        ratioPeriodSpin->setMaximum(1000);
        ratioPeriodSpin->setValue(10);

        contiForm->setWidget(16, QFormLayout::FieldRole, ratioPeriodSpin);

        markOffsetLabel = new QLabel(contiGroup);
        markOffsetLabel->setObjectName("markOffsetLabel");

        contiForm->setWidget(17, QFormLayout::LabelRole, markOffsetLabel);

        markOffsetSpin = new QSpinBox(contiGroup);
        markOffsetSpin->setObjectName("markOffsetSpin");
        markOffsetSpin->setMinimum(-20);
        markOffsetSpin->setMaximum(20);

        contiForm->setWidget(17, QFormLayout::FieldRole, markOffsetSpin);

        bufferGainLabel = new QLabel(contiGroup);
        bufferGainLabel->setObjectName("bufferGainLabel");

        contiForm->setWidget(18, QFormLayout::LabelRole, bufferGainLabel);

        bufferGainSpin = new QDoubleSpinBox(contiGroup);
        bufferGainSpin->setObjectName("bufferGainSpin");
        bufferGainSpin->setDecimals(3);
        bufferGainSpin->setMaximum(2.000000000000000);
        bufferGainSpin->setSingleStep(0.010000000000000);
        bufferGainSpin->setValue(0.100000000000000);

        contiForm->setWidget(18, QFormLayout::FieldRole, bufferGainSpin);

        executionDelayLabel = new QLabel(contiGroup);
        executionDelayLabel->setObjectName("executionDelayLabel");

        contiForm->setWidget(19, QFormLayout::LabelRole, executionDelayLabel);

        executionDelaySpin = new QSpinBox(contiGroup);
        executionDelaySpin->setObjectName("executionDelaySpin");
        executionDelaySpin->setMinimum(10);
        executionDelaySpin->setMaximum(5000);
        executionDelaySpin->setValue(250);

        contiForm->setWidget(19, QFormLayout::FieldRole, executionDelaySpin);

        phaseDeadbandLabel = new QLabel(contiGroup);
        phaseDeadbandLabel->setObjectName("phaseDeadbandLabel");

        contiForm->setWidget(20, QFormLayout::LabelRole, phaseDeadbandLabel);

        phaseDeadbandSpin = new QSpinBox(contiGroup);
        phaseDeadbandSpin->setObjectName("phaseDeadbandSpin");
        phaseDeadbandSpin->setMaximum(1000);
        phaseDeadbandSpin->setValue(20);

        contiForm->setWidget(20, QFormLayout::FieldRole, phaseDeadbandSpin);

        ratioApiIntervalLabel = new QLabel(contiGroup);
        ratioApiIntervalLabel->setObjectName("ratioApiIntervalLabel");

        contiForm->setWidget(21, QFormLayout::LabelRole, ratioApiIntervalLabel);

        ratioApiIntervalSpin = new QSpinBox(contiGroup);
        ratioApiIntervalSpin->setObjectName("ratioApiIntervalSpin");
        ratioApiIntervalSpin->setMinimum(10);
        ratioApiIntervalSpin->setMaximum(5000);
        ratioApiIntervalSpin->setValue(100);

        contiForm->setWidget(21, QFormLayout::FieldRole, ratioApiIntervalSpin);

        ratioSafetyApiIntervalLabel = new QLabel(contiGroup);
        ratioSafetyApiIntervalLabel->setObjectName("ratioSafetyApiIntervalLabel");

        contiForm->setWidget(22, QFormLayout::LabelRole, ratioSafetyApiIntervalLabel);

        ratioSafetyApiIntervalSpin = new QSpinBox(contiGroup);
        ratioSafetyApiIntervalSpin->setObjectName("ratioSafetyApiIntervalSpin");
        ratioSafetyApiIntervalSpin->setMinimum(10);
        ratioSafetyApiIntervalSpin->setMaximum(5000);
        ratioSafetyApiIntervalSpin->setValue(50);

        contiForm->setWidget(22, QFormLayout::FieldRole, ratioSafetyApiIntervalSpin);


        settingsLayout->addWidget(contiGroup);


        contiTestLayout->addLayout(settingsLayout);

        contiButtonLayout = new QHBoxLayout();
        contiButtonLayout->setObjectName("contiButtonLayout");
        startButton = new QPushButton(contiTestScrollContent);
        startButton->setObjectName("startButton");

        contiButtonLayout->addWidget(startButton);

        stopButton = new QPushButton(contiTestScrollContent);
        stopButton->setObjectName("stopButton");

        contiButtonLayout->addWidget(stopButton);


        contiTestLayout->addLayout(contiButtonLayout);

        contiTrajectoryGroup = new QGroupBox(contiTestScrollContent);
        contiTrajectoryGroup->setObjectName("contiTrajectoryGroup");
        contiTrajectoryLayout = new QVBoxLayout(contiTrajectoryGroup);
        contiTrajectoryLayout->setObjectName("contiTrajectoryLayout");
        contiTrajectoryChartView = new ZoomableChartView(contiTrajectoryGroup);
        contiTrajectoryChartView->setObjectName("contiTrajectoryChartView");
        contiTrajectoryChartView->setMinimumSize(QSize(0, 300));

        contiTrajectoryLayout->addWidget(contiTrajectoryChartView);


        contiTestLayout->addWidget(contiTrajectoryGroup);

        contiTestScrollArea->setWidget(contiTestScrollContent);

        contiTestOuterLayout->addWidget(contiTestScrollArea);

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
        startRecordingButton->setEnabled(false);

        telemetryRecordLayout->addWidget(startRecordingButton, 0, 0, 1, 1);

        stopRecordingButton = new QPushButton(telemetryRecordGroup);
        stopRecordingButton->setObjectName("stopRecordingButton");
        stopRecordingButton->setEnabled(false);

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
        positionChartView = new ZoomableChartView(telemetryPlotSplitter);
        positionChartView->setObjectName("positionChartView");
        positionChartView->setMinimumSize(QSize(0, 240));
        telemetryPlotSplitter->addWidget(positionChartView);
        followingErrorChartView = new ZoomableChartView(telemetryPlotSplitter);
        followingErrorChartView->setObjectName("followingErrorChartView");
        followingErrorChartView->setMinimumSize(QSize(0, 240));
        telemetryPlotSplitter->addWidget(followingErrorChartView);

        telemetryLayout->addWidget(telemetryPlotSplitter);

        telemetryHintLabel = new QLabel(telemetryTab);
        telemetryHintLabel->setObjectName("telemetryHintLabel");
        telemetryHintLabel->setWordWrap(true);

        telemetryLayout->addWidget(telemetryHintLabel);

        tabWidget->addTab(telemetryTab, QString());
        velocityControlTab = new QWidget();
        velocityControlTab->setObjectName("velocityControlTab");
        velocityControlTabLayout = new QVBoxLayout(velocityControlTab);
        velocityControlTabLayout->setObjectName("velocityControlTabLayout");
        velocityControlScrollArea = new QScrollArea(velocityControlTab);
        velocityControlScrollArea->setObjectName("velocityControlScrollArea");
        velocityControlScrollArea->setWidgetResizable(true);
        velocityControlScrollContent = new QWidget();
        velocityControlScrollContent->setObjectName("velocityControlScrollContent");
        velocityControlScrollContent->setGeometry(QRect(0, 0, 781, 1136));
        velocityControlContentLayout = new QVBoxLayout(velocityControlScrollContent);
        velocityControlContentLayout->setObjectName("velocityControlContentLayout");
        velocityControlParameterGroup = new QGroupBox(velocityControlScrollContent);
        velocityControlParameterGroup->setObjectName("velocityControlParameterGroup");
        velocityControlParameterLayout = new QGridLayout(velocityControlParameterGroup);
        velocityControlParameterLayout->setObjectName("velocityControlParameterLayout");
        velocityTrajectoryGroup = new QGroupBox(velocityControlParameterGroup);
        velocityTrajectoryGroup->setObjectName("velocityTrajectoryGroup");
        velocityTrajectoryLayout = new QGridLayout(velocityTrajectoryGroup);
        velocityTrajectoryLayout->setObjectName("velocityTrajectoryLayout");
        velocityAxisLabel = new QLabel(velocityTrajectoryGroup);
        velocityAxisLabel->setObjectName("velocityAxisLabel");

        velocityTrajectoryLayout->addWidget(velocityAxisLabel, 0, 0, 1, 1);

        velocityAxisCombo = new QComboBox(velocityTrajectoryGroup);
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->addItem(QString());
        velocityAxisCombo->setObjectName("velocityAxisCombo");

        velocityTrajectoryLayout->addWidget(velocityAxisCombo, 0, 1, 1, 1);

        velocityDeltaLabel = new QLabel(velocityTrajectoryGroup);
        velocityDeltaLabel->setObjectName("velocityDeltaLabel");

        velocityTrajectoryLayout->addWidget(velocityDeltaLabel, 0, 2, 1, 1);

        velocityDeltaSpin = new QDoubleSpinBox(velocityTrajectoryGroup);
        velocityDeltaSpin->setObjectName("velocityDeltaSpin");
        velocityDeltaSpin->setDecimals(4);
        velocityDeltaSpin->setMinimum(-1000000.000000000000000);
        velocityDeltaSpin->setMaximum(1000000.000000000000000);
        velocityDeltaSpin->setValue(30.000000000000000);

        velocityTrajectoryLayout->addWidget(velocityDeltaSpin, 0, 3, 1, 1);

        velocityDurationLabel = new QLabel(velocityTrajectoryGroup);
        velocityDurationLabel->setObjectName("velocityDurationLabel");

        velocityTrajectoryLayout->addWidget(velocityDurationLabel, 1, 2, 1, 1);

        velocityDurationSpin = new QDoubleSpinBox(velocityTrajectoryGroup);
        velocityDurationSpin->setObjectName("velocityDurationSpin");
        velocityDurationSpin->setDecimals(3);
        velocityDurationSpin->setMinimum(0.100000000000000);
        velocityDurationSpin->setMaximum(3600.000000000000000);
        velocityDurationSpin->setValue(5.000000000000000);

        velocityTrajectoryLayout->addWidget(velocityDurationSpin, 1, 3, 1, 1);

        velocityControlPeriodLabel = new QLabel(velocityTrajectoryGroup);
        velocityControlPeriodLabel->setObjectName("velocityControlPeriodLabel");

        velocityTrajectoryLayout->addWidget(velocityControlPeriodLabel, 1, 0, 1, 1);

        velocityControlPeriodSpin = new QSpinBox(velocityTrajectoryGroup);
        velocityControlPeriodSpin->setObjectName("velocityControlPeriodSpin");
        velocityControlPeriodSpin->setMinimum(1);
        velocityControlPeriodSpin->setMaximum(100);
        velocityControlPeriodSpin->setValue(10);

        velocityTrajectoryLayout->addWidget(velocityControlPeriodSpin, 1, 1, 1, 1);

        velocityTrajectorySpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        velocityTrajectoryLayout->addItem(velocityTrajectorySpacer, 0, 4, 2, 1);


        velocityControlParameterLayout->addWidget(velocityTrajectoryGroup, 0, 0, 1, 1);

        velocityClosedLoopGroup = new QGroupBox(velocityControlParameterGroup);
        velocityClosedLoopGroup->setObjectName("velocityClosedLoopGroup");
        velocityClosedLoopLayout = new QGridLayout(velocityClosedLoopGroup);
        velocityClosedLoopLayout->setObjectName("velocityClosedLoopLayout");
        velocityFeedforwardCheck = new QCheckBox(velocityClosedLoopGroup);
        velocityFeedforwardCheck->setObjectName("velocityFeedforwardCheck");
        velocityFeedforwardCheck->setChecked(true);

        velocityClosedLoopLayout->addWidget(velocityFeedforwardCheck, 0, 0, 1, 1);

        velocityFeedforwardGainSpin = new QDoubleSpinBox(velocityClosedLoopGroup);
        velocityFeedforwardGainSpin->setObjectName("velocityFeedforwardGainSpin");
        velocityFeedforwardGainSpin->setDecimals(3);
        velocityFeedforwardGainSpin->setMinimum(0.000000000000000);
        velocityFeedforwardGainSpin->setMaximum(5.000000000000000);
        velocityFeedforwardGainSpin->setValue(1.000000000000000);

        velocityClosedLoopLayout->addWidget(velocityFeedforwardGainSpin, 0, 1, 1, 1);

        velocityPidEnableCheck = new QCheckBox(velocityClosedLoopGroup);
        velocityPidEnableCheck->setObjectName("velocityPidEnableCheck");
        velocityPidEnableCheck->setChecked(true);

        velocityClosedLoopLayout->addWidget(velocityPidEnableCheck, 0, 2, 1, 1);

        velocityKpLabel = new QLabel(velocityClosedLoopGroup);
        velocityKpLabel->setObjectName("velocityKpLabel");

        velocityClosedLoopLayout->addWidget(velocityKpLabel, 1, 0, 1, 1);

        velocityKpSpin = new QDoubleSpinBox(velocityClosedLoopGroup);
        velocityKpSpin->setObjectName("velocityKpSpin");
        velocityKpSpin->setDecimals(5);
        velocityKpSpin->setMaximum(1000.000000000000000);

        velocityClosedLoopLayout->addWidget(velocityKpSpin, 1, 1, 1, 1);

        velocityKiLabel = new QLabel(velocityClosedLoopGroup);
        velocityKiLabel->setObjectName("velocityKiLabel");

        velocityClosedLoopLayout->addWidget(velocityKiLabel, 1, 2, 1, 1);

        velocityKiSpin = new QDoubleSpinBox(velocityClosedLoopGroup);
        velocityKiSpin->setObjectName("velocityKiSpin");
        velocityKiSpin->setDecimals(5);
        velocityKiSpin->setMaximum(1000.000000000000000);

        velocityClosedLoopLayout->addWidget(velocityKiSpin, 1, 3, 1, 1);

        velocityKdLabel = new QLabel(velocityClosedLoopGroup);
        velocityKdLabel->setObjectName("velocityKdLabel");

        velocityClosedLoopLayout->addWidget(velocityKdLabel, 2, 0, 1, 1);

        velocityKdSpin = new QDoubleSpinBox(velocityClosedLoopGroup);
        velocityKdSpin->setObjectName("velocityKdSpin");
        velocityKdSpin->setDecimals(5);
        velocityKdSpin->setMaximum(1000.000000000000000);

        velocityClosedLoopLayout->addWidget(velocityKdSpin, 2, 1, 1, 1);

        velocityIntegralLimitLabel = new QLabel(velocityClosedLoopGroup);
        velocityIntegralLimitLabel->setObjectName("velocityIntegralLimitLabel");

        velocityClosedLoopLayout->addWidget(velocityIntegralLimitLabel, 2, 2, 1, 1);

        velocityIntegralLimitSpin = new QDoubleSpinBox(velocityClosedLoopGroup);
        velocityIntegralLimitSpin->setObjectName("velocityIntegralLimitSpin");
        velocityIntegralLimitSpin->setDecimals(3);
        velocityIntegralLimitSpin->setMaximum(100000.000000000000000);
        velocityIntegralLimitSpin->setValue(10.000000000000000);

        velocityClosedLoopLayout->addWidget(velocityIntegralLimitSpin, 2, 3, 1, 1);

        velocityMaxCorrectionLabel = new QLabel(velocityClosedLoopGroup);
        velocityMaxCorrectionLabel->setObjectName("velocityMaxCorrectionLabel");

        velocityClosedLoopLayout->addWidget(velocityMaxCorrectionLabel, 3, 0, 1, 1);

        velocityMaxCorrectionSpin = new QDoubleSpinBox(velocityClosedLoopGroup);
        velocityMaxCorrectionSpin->setObjectName("velocityMaxCorrectionSpin");
        velocityMaxCorrectionSpin->setDecimals(3);
        velocityMaxCorrectionSpin->setMaximum(10000.000000000000000);
        velocityMaxCorrectionSpin->setValue(20.000000000000000);

        velocityClosedLoopLayout->addWidget(velocityMaxCorrectionSpin, 3, 1, 1, 1);

        velocityClosedLoopSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        velocityClosedLoopLayout->addItem(velocityClosedLoopSpacer, 0, 4, 4, 1);


        velocityControlParameterLayout->addWidget(velocityClosedLoopGroup, 0, 1, 1, 1);

        velocityLimitGroup = new QGroupBox(velocityControlParameterGroup);
        velocityLimitGroup->setObjectName("velocityLimitGroup");
        velocityLimitLayout = new QGridLayout(velocityLimitGroup);
        velocityLimitLayout->setObjectName("velocityLimitLayout");
        velocityMaxSpeedLabel = new QLabel(velocityLimitGroup);
        velocityMaxSpeedLabel->setObjectName("velocityMaxSpeedLabel");

        velocityLimitLayout->addWidget(velocityMaxSpeedLabel, 0, 0, 1, 1);

        velocityMaxSpeedSpin = new QDoubleSpinBox(velocityLimitGroup);
        velocityMaxSpeedSpin->setObjectName("velocityMaxSpeedSpin");
        velocityMaxSpeedSpin->setDecimals(3);
        velocityMaxSpeedSpin->setMinimum(0.001000000000000);
        velocityMaxSpeedSpin->setMaximum(10000.000000000000000);
        velocityMaxSpeedSpin->setValue(30.000000000000000);

        velocityLimitLayout->addWidget(velocityMaxSpeedSpin, 0, 1, 1, 1);

        velocityMaxAccelerationLabel = new QLabel(velocityLimitGroup);
        velocityMaxAccelerationLabel->setObjectName("velocityMaxAccelerationLabel");

        velocityLimitLayout->addWidget(velocityMaxAccelerationLabel, 0, 2, 1, 1);

        velocityMaxAccelerationSpin = new QDoubleSpinBox(velocityLimitGroup);
        velocityMaxAccelerationSpin->setObjectName("velocityMaxAccelerationSpin");
        velocityMaxAccelerationSpin->setDecimals(3);
        velocityMaxAccelerationSpin->setMinimum(0.001000000000000);
        velocityMaxAccelerationSpin->setMaximum(100000.000000000000000);
        velocityMaxAccelerationSpin->setValue(100.000000000000000);

        velocityLimitLayout->addWidget(velocityMaxAccelerationSpin, 0, 3, 1, 1);

        velocityChangeTimeLabel = new QLabel(velocityLimitGroup);
        velocityChangeTimeLabel->setObjectName("velocityChangeTimeLabel");

        velocityLimitLayout->addWidget(velocityChangeTimeLabel, 1, 0, 1, 1);

        velocityChangeTimeSpin = new QDoubleSpinBox(velocityLimitGroup);
        velocityChangeTimeSpin->setObjectName("velocityChangeTimeSpin");
        velocityChangeTimeSpin->setDecimals(3);
        velocityChangeTimeSpin->setMinimum(0.000000000000000);
        velocityChangeTimeSpin->setMaximum(1.000000000000000);
        velocityChangeTimeSpin->setSingleStep(0.010000000000000);
        velocityChangeTimeSpin->setValue(0.010000000000000);

        velocityLimitLayout->addWidget(velocityChangeTimeSpin, 1, 1, 1, 1);

        velocityStartThresholdLabel = new QLabel(velocityLimitGroup);
        velocityStartThresholdLabel->setObjectName("velocityStartThresholdLabel");

        velocityLimitLayout->addWidget(velocityStartThresholdLabel, 1, 2, 1, 1);

        velocityStartThresholdSpin = new QDoubleSpinBox(velocityLimitGroup);
        velocityStartThresholdSpin->setObjectName("velocityStartThresholdSpin");
        velocityStartThresholdSpin->setDecimals(4);
        velocityStartThresholdSpin->setMinimum(0.000100000000000);
        velocityStartThresholdSpin->setMaximum(1000.000000000000000);
        velocityStartThresholdSpin->setValue(0.020000000000000);

        velocityLimitLayout->addWidget(velocityStartThresholdSpin, 1, 3, 1, 1);

        velocityLimitSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        velocityLimitLayout->addItem(velocityLimitSpacer, 0, 4, 2, 1);


        velocityControlParameterLayout->addWidget(velocityLimitGroup, 1, 0, 1, 1);

        velocityCriterionGroup = new QGroupBox(velocityControlParameterGroup);
        velocityCriterionGroup->setObjectName("velocityCriterionGroup");
        velocityCriterionLayout = new QGridLayout(velocityCriterionGroup);
        velocityCriterionLayout->setObjectName("velocityCriterionLayout");
        velocityPositionToleranceLabel = new QLabel(velocityCriterionGroup);
        velocityPositionToleranceLabel->setObjectName("velocityPositionToleranceLabel");

        velocityCriterionLayout->addWidget(velocityPositionToleranceLabel, 0, 0, 1, 1);

        velocityPositionToleranceSpin = new QDoubleSpinBox(velocityCriterionGroup);
        velocityPositionToleranceSpin->setObjectName("velocityPositionToleranceSpin");
        velocityPositionToleranceSpin->setDecimals(4);
        velocityPositionToleranceSpin->setMinimum(0.000100000000000);
        velocityPositionToleranceSpin->setMaximum(1000.000000000000000);
        velocityPositionToleranceSpin->setValue(0.050000000000000);

        velocityCriterionLayout->addWidget(velocityPositionToleranceSpin, 0, 1, 1, 1);

        velocitySpeedToleranceLabel = new QLabel(velocityCriterionGroup);
        velocitySpeedToleranceLabel->setObjectName("velocitySpeedToleranceLabel");

        velocityCriterionLayout->addWidget(velocitySpeedToleranceLabel, 0, 2, 1, 1);

        velocitySpeedToleranceSpin = new QDoubleSpinBox(velocityCriterionGroup);
        velocitySpeedToleranceSpin->setObjectName("velocitySpeedToleranceSpin");
        velocitySpeedToleranceSpin->setDecimals(3);
        velocitySpeedToleranceSpin->setMinimum(0.001000000000000);
        velocitySpeedToleranceSpin->setMaximum(1000.000000000000000);
        velocitySpeedToleranceSpin->setValue(0.200000000000000);

        velocityCriterionLayout->addWidget(velocitySpeedToleranceSpin, 0, 3, 1, 1);

        velocityStableDwellLabel = new QLabel(velocityCriterionGroup);
        velocityStableDwellLabel->setObjectName("velocityStableDwellLabel");

        velocityCriterionLayout->addWidget(velocityStableDwellLabel, 1, 0, 1, 1);

        velocityStableDwellSpin = new QSpinBox(velocityCriterionGroup);
        velocityStableDwellSpin->setObjectName("velocityStableDwellSpin");
        velocityStableDwellSpin->setMinimum(10);
        velocityStableDwellSpin->setMaximum(10000);
        velocityStableDwellSpin->setValue(200);

        velocityCriterionLayout->addWidget(velocityStableDwellSpin, 1, 1, 1, 1);

        velocityFinishTimeoutLabel = new QLabel(velocityCriterionGroup);
        velocityFinishTimeoutLabel->setObjectName("velocityFinishTimeoutLabel");

        velocityCriterionLayout->addWidget(velocityFinishTimeoutLabel, 1, 2, 1, 1);

        velocityFinishTimeoutSpin = new QSpinBox(velocityCriterionGroup);
        velocityFinishTimeoutSpin->setObjectName("velocityFinishTimeoutSpin");
        velocityFinishTimeoutSpin->setMinimum(10);
        velocityFinishTimeoutSpin->setMaximum(60000);
        velocityFinishTimeoutSpin->setValue(3000);

        velocityCriterionLayout->addWidget(velocityFinishTimeoutSpin, 1, 3, 1, 1);

        velocityMaxFollowingErrorLabel = new QLabel(velocityCriterionGroup);
        velocityMaxFollowingErrorLabel->setObjectName("velocityMaxFollowingErrorLabel");

        velocityCriterionLayout->addWidget(velocityMaxFollowingErrorLabel, 2, 0, 1, 1);

        velocityMaxFollowingErrorSpin = new QDoubleSpinBox(velocityCriterionGroup);
        velocityMaxFollowingErrorSpin->setObjectName("velocityMaxFollowingErrorSpin");
        velocityMaxFollowingErrorSpin->setDecimals(3);
        velocityMaxFollowingErrorSpin->setMinimum(0.001000000000000);
        velocityMaxFollowingErrorSpin->setMaximum(100000.000000000000000);
        velocityMaxFollowingErrorSpin->setValue(10.000000000000000);

        velocityCriterionLayout->addWidget(velocityMaxFollowingErrorSpin, 2, 1, 1, 1);

        velocityTraceTimeoutLabel = new QLabel(velocityCriterionGroup);
        velocityTraceTimeoutLabel->setObjectName("velocityTraceTimeoutLabel");

        velocityCriterionLayout->addWidget(velocityTraceTimeoutLabel, 2, 2, 1, 1);

        velocityTraceTimeoutSpin = new QSpinBox(velocityCriterionGroup);
        velocityTraceTimeoutSpin->setObjectName("velocityTraceTimeoutSpin");
        velocityTraceTimeoutSpin->setMinimum(10);
        velocityTraceTimeoutSpin->setMaximum(10000);
        velocityTraceTimeoutSpin->setValue(100);

        velocityCriterionLayout->addWidget(velocityTraceTimeoutSpin, 2, 3, 1, 1);

        velocityCriterionSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        velocityCriterionLayout->addItem(velocityCriterionSpacer, 0, 4, 3, 1);


        velocityControlParameterLayout->addWidget(velocityCriterionGroup, 1, 1, 1, 1);


        velocityControlContentLayout->addWidget(velocityControlParameterGroup);

        velocityControlOperationGroup = new QGroupBox(velocityControlScrollContent);
        velocityControlOperationGroup->setObjectName("velocityControlOperationGroup");
        velocityControlOperationLayout = new QGridLayout(velocityControlOperationGroup);
        velocityControlOperationLayout->setObjectName("velocityControlOperationLayout");
        velocityEnableAxisButton = new QPushButton(velocityControlOperationGroup);
        velocityEnableAxisButton->setObjectName("velocityEnableAxisButton");

        velocityControlOperationLayout->addWidget(velocityEnableAxisButton, 0, 0, 1, 1);

        velocityDisableAxisButton = new QPushButton(velocityControlOperationGroup);
        velocityDisableAxisButton->setObjectName("velocityDisableAxisButton");

        velocityControlOperationLayout->addWidget(velocityDisableAxisButton, 0, 1, 1, 1);

        velocityResetButton = new QPushButton(velocityControlOperationGroup);
        velocityResetButton->setObjectName("velocityResetButton");

        velocityControlOperationLayout->addWidget(velocityResetButton, 0, 2, 1, 1);

        velocityStartButton = new QPushButton(velocityControlOperationGroup);
        velocityStartButton->setObjectName("velocityStartButton");

        velocityControlOperationLayout->addWidget(velocityStartButton, 0, 3, 1, 1);

        velocityStopButton = new QPushButton(velocityControlOperationGroup);
        velocityStopButton->setObjectName("velocityStopButton");

        velocityControlOperationLayout->addWidget(velocityStopButton, 0, 4, 1, 1);

        velocityEmergencyStopButton = new QPushButton(velocityControlOperationGroup);
        velocityEmergencyStopButton->setObjectName("velocityEmergencyStopButton");
        velocityEmergencyStopButton->setStyleSheet(QString::fromUtf8("background-color:#b00020;color:white;font-weight:bold;"));

        velocityControlOperationLayout->addWidget(velocityEmergencyStopButton, 0, 5, 1, 1);

        velocityClearCurvesButton = new QPushButton(velocityControlOperationGroup);
        velocityClearCurvesButton->setObjectName("velocityClearCurvesButton");

        velocityControlOperationLayout->addWidget(velocityClearCurvesButton, 0, 6, 1, 1);

        velocityStateLabel = new QLabel(velocityControlOperationGroup);
        velocityStateLabel->setObjectName("velocityStateLabel");

        velocityControlOperationLayout->addWidget(velocityStateLabel, 1, 0, 1, 1);

        velocityStateValueLabel = new QLabel(velocityControlOperationGroup);
        velocityStateValueLabel->setObjectName("velocityStateValueLabel");

        velocityControlOperationLayout->addWidget(velocityStateValueLabel, 1, 1, 1, 2);

        velocityTimingLabel = new QLabel(velocityControlOperationGroup);
        velocityTimingLabel->setObjectName("velocityTimingLabel");

        velocityControlOperationLayout->addWidget(velocityTimingLabel, 1, 3, 1, 1);

        velocityTimingValueLabel = new QLabel(velocityControlOperationGroup);
        velocityTimingValueLabel->setObjectName("velocityTimingValueLabel");

        velocityControlOperationLayout->addWidget(velocityTimingValueLabel, 1, 4, 1, 3);

        velocityPositionStatusLabel = new QLabel(velocityControlOperationGroup);
        velocityPositionStatusLabel->setObjectName("velocityPositionStatusLabel");

        velocityControlOperationLayout->addWidget(velocityPositionStatusLabel, 2, 0, 1, 1);

        velocityPositionStatusValueLabel = new QLabel(velocityControlOperationGroup);
        velocityPositionStatusValueLabel->setObjectName("velocityPositionStatusValueLabel");

        velocityControlOperationLayout->addWidget(velocityPositionStatusValueLabel, 2, 1, 1, 6);

        velocitySpeedStatusLabel = new QLabel(velocityControlOperationGroup);
        velocitySpeedStatusLabel->setObjectName("velocitySpeedStatusLabel");

        velocityControlOperationLayout->addWidget(velocitySpeedStatusLabel, 3, 0, 1, 1);

        velocitySpeedStatusValueLabel = new QLabel(velocityControlOperationGroup);
        velocitySpeedStatusValueLabel->setObjectName("velocitySpeedStatusValueLabel");

        velocityControlOperationLayout->addWidget(velocitySpeedStatusValueLabel, 3, 1, 1, 6);

        velocityPidStatusLabel = new QLabel(velocityControlOperationGroup);
        velocityPidStatusLabel->setObjectName("velocityPidStatusLabel");

        velocityControlOperationLayout->addWidget(velocityPidStatusLabel, 4, 0, 1, 1);

        velocityPidStatusValueLabel = new QLabel(velocityControlOperationGroup);
        velocityPidStatusValueLabel->setObjectName("velocityPidStatusValueLabel");

        velocityControlOperationLayout->addWidget(velocityPidStatusValueLabel, 4, 1, 1, 6);


        velocityControlContentLayout->addWidget(velocityControlOperationGroup);

        velocityControlChartSplitter = new QSplitter(velocityControlScrollContent);
        velocityControlChartSplitter->setObjectName("velocityControlChartSplitter");
        velocityControlChartSplitter->setOrientation(Qt::Orientation::Vertical);
        velocityControlChartSplitter->setChildrenCollapsible(false);
        velocityPositionChartView = new ZoomableChartView(velocityControlChartSplitter);
        velocityPositionChartView->setObjectName("velocityPositionChartView");
        velocityPositionChartView->setMinimumSize(QSize(0, 240));
        velocityControlChartSplitter->addWidget(velocityPositionChartView);
        velocityErrorChartView = new ZoomableChartView(velocityControlChartSplitter);
        velocityErrorChartView->setObjectName("velocityErrorChartView");
        velocityErrorChartView->setMinimumSize(QSize(0, 220));
        velocityControlChartSplitter->addWidget(velocityErrorChartView);
        velocitySpeedChartView = new ZoomableChartView(velocityControlChartSplitter);
        velocitySpeedChartView->setObjectName("velocitySpeedChartView");
        velocitySpeedChartView->setMinimumSize(QSize(0, 240));
        velocityControlChartSplitter->addWidget(velocitySpeedChartView);

        velocityControlContentLayout->addWidget(velocityControlChartSplitter);

        velocityControlHintLabel = new QLabel(velocityControlScrollContent);
        velocityControlHintLabel->setObjectName("velocityControlHintLabel");
        velocityControlHintLabel->setWordWrap(true);

        velocityControlContentLayout->addWidget(velocityControlHintLabel);

        velocityControlScrollArea->setWidget(velocityControlScrollContent);

        velocityControlTabLayout->addWidget(velocityControlScrollArea);

        tabWidget->addTab(velocityControlTab, QString());
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
        copyLogButton = new QPushButton(logGroup);
        copyLogButton->setObjectName("copyLogButton");

        logToolbarLayout->addWidget(copyLogButton);

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

        busCycleCombo->setCurrentIndex(2);
        tabWidget->setCurrentIndex(3);
        holdAxisCombo->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "E5000 \350\277\220\345\212\250\346\216\247\345\210\266\346\265\213\350\257\225\345\271\263\345\217\260", nullptr));
        globalControlGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241\344\270\216\345\205\250\345\261\200\345\256\211\345\205\250", nullptr));
        globalStatusGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241\347\212\266\346\200\201", nullptr));
        stateLabel->setText(QCoreApplication::translate("MainWindow", "\347\250\213\345\272\217\347\212\266\346\200\201", nullptr));
        stateValueLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\252\345\210\235\345\247\213\345\214\226", nullptr));
#if QT_CONFIG(tooltip)
        busCycleSettingLabel->setToolTip(QCoreApplication::translate("MainWindow", "\346\211\213\345\206\214\346\224\257\346\214\201 250\343\200\201500\343\200\2011000\343\200\2012000 \316\274s\357\274\233\344\273\205\345\234\250\345\210\235\345\247\213\345\214\226\346\216\247\345\210\266\345\215\241\346\227\266\345\206\231\345\205\245\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        busCycleSettingLabel->setText(QCoreApplication::translate("MainWindow", "\346\200\273\347\272\277\345\221\250\346\234\237\357\274\210\350\256\276\347\275\256\357\274\211", nullptr));
        busCycleCombo->setItemText(0, QCoreApplication::translate("MainWindow", "250 \316\274s", nullptr));
        busCycleCombo->setItemText(1, QCoreApplication::translate("MainWindow", "500 \316\274s", nullptr));
        busCycleCombo->setItemText(2, QCoreApplication::translate("MainWindow", "1000 \316\274s", nullptr));
        busCycleCombo->setItemText(3, QCoreApplication::translate("MainWindow", "2000 \316\274s", nullptr));

#if QT_CONFIG(tooltip)
        busCycleCombo->setToolTip(QCoreApplication::translate("MainWindow", "EtherCAT \346\200\273\347\272\277\345\221\250\346\234\237\357\274\214\345\217\257\351\200\211 250 / 500 / 1000 / 2000 \316\274s\343\200\202\344\277\256\346\224\271\345\220\216\351\234\200\345\205\263\351\227\255\345\271\266\351\207\215\346\226\260\345\210\235\345\247\213\345\214\226\346\216\247\345\210\266\345\215\241\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        busCycleReadLabel->setText(QCoreApplication::translate("MainWindow", "\346\200\273\347\272\277\345\221\250\346\234\237\357\274\210\350\257\273\345\233\236\357\274\211", nullptr));
        busCycleReadValueLabel->setText(QCoreApplication::translate("MainWindow", "\345\276\205\345\210\235\345\247\213\345\214\226", nullptr));
        readBusCycleButton->setText(QCoreApplication::translate("MainWindow", "\345\210\267\346\226\260", nullptr));
        traceSampleLabel->setText(QCoreApplication::translate("MainWindow", "Trace \351\207\207\346\240\267\345\221\250\346\234\237", nullptr));
        traceSampleValueLabel->setText(QCoreApplication::translate("MainWindow", "\345\276\205\345\210\235\345\247\213\345\214\226", nullptr));
        globalActionGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\215\241\346\223\215\344\275\234\344\270\216\350\275\264\347\212\266\346\200\201", nullptr));
        initializeButton->setText(QCoreApplication::translate("MainWindow", "\345\210\235\345\247\213\345\214\226\346\216\247\345\210\266\345\215\241", nullptr));
        closeBoardButton->setText(QCoreApplication::translate("MainWindow", "\345\256\211\345\205\250\345\205\263\351\227\255\346\216\247\345\210\266\345\215\241", nullptr));
        axisEnableStatusGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\200\273\347\272\277\350\275\264\344\275\277\350\203\275\347\212\266\346\200\201", nullptr));
        axis0EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2640 \346\234\252\347\237\245", nullptr));
        axis1EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2641 \346\234\252\347\237\245", nullptr));
        axis2EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2642 \346\234\252\347\237\245", nullptr));
        axis3EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2643 \346\234\252\347\237\245", nullptr));
        axis4EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2644 \346\234\252\347\237\245", nullptr));
        axis5EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2645 \346\234\252\347\237\245", nullptr));
        axis6EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2646 \346\234\252\347\237\245", nullptr));
        axis7EnableStateLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\2647 \346\234\252\347\237\245", nullptr));
        enableAllAxesButton->setText(QCoreApplication::translate("MainWindow", "\344\270\200\351\224\256\344\275\277\350\203\275\345\205\250\351\203\250\345\234\250\347\272\277\350\275\264", nullptr));
        disableAllAxesButton->setText(QCoreApplication::translate("MainWindow", "\344\270\200\351\224\256\345\244\261\350\203\275\345\205\250\351\203\250\345\234\250\347\272\277\350\275\264", nullptr));
        globalEmergencyGroup->setTitle(QCoreApplication::translate("MainWindow", "\347\264\247\346\200\245\346\223\215\344\275\234", nullptr));
        emergencyStopButton->setText(QCoreApplication::translate("MainWindow", "\345\205\250\345\261\200\347\253\213\345\215\263\345\201\234\346\255\242", nullptr));
        globalEmergencyHintLabel->setText(QCoreApplication::translate("MainWindow", "\347\253\213\345\215\263\345\201\234\346\255\242\345\275\223\345\211\215\346\255\243\345\234\250\346\211\247\350\241\214\347\232\204\350\277\236\347\273\255\346\217\222\350\241\245\343\200\201\345\215\225\350\275\264\347\202\271\344\275\215\346\210\226\351\200\237\345\272\246\351\227\255\347\216\257\350\277\220\345\212\250\343\200\202", nullptr));
        contiRuntimeStatusGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\217\222\350\241\245\350\275\264\344\270\216\350\277\220\350\241\214\347\212\266\346\200\201", nullptr));
#if QT_CONFIG(tooltip)
        contiEnableAxesButton->setToolTip(QCoreApplication::translate("MainWindow", "\344\275\277\350\203\275\346\234\254\351\241\265\342\200\234\344\270\273\345\212\250\350\275\264\342\200\235\345\222\214\342\200\234\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264\342\200\235\346\211\200\351\200\211\347\232\204\346\265\213\350\257\225\350\275\264\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        contiEnableAxesButton->setText(QCoreApplication::translate("MainWindow", "\344\275\277\350\203\275\346\217\222\350\241\245\346\265\213\350\257\225\350\275\264", nullptr));
#if QT_CONFIG(tooltip)
        contiDisableAxesButton->setToolTip(QCoreApplication::translate("MainWindow", "\345\244\261\350\203\275\346\234\254\351\241\265\342\200\234\344\270\273\345\212\250\350\275\264\342\200\235\345\222\214\342\200\234\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264\342\200\235\346\211\200\351\200\211\347\232\204\346\265\213\350\257\225\350\275\264\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        contiDisableAxesButton->setText(QCoreApplication::translate("MainWindow", "\345\244\261\350\203\275\346\217\222\350\241\245\346\265\213\350\257\225\350\275\264", nullptr));
        contiRunStateLabel->setText(QCoreApplication::translate("MainWindow", "runState", nullptr));
        contiRunStateValueLabel->setText(QCoreApplication::translate("MainWindow", "4", nullptr));
        contiMarkLabel->setText(QCoreApplication::translate("MainWindow", "\345\275\223\345\211\215 / \345\267\262\346\216\250\351\200\201 mark", nullptr));
        contiMarkValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0", nullptr));
        contiBufferLabel->setText(QCoreApplication::translate("MainWindow", "\347\274\223\345\206\262\346\256\265\346\225\260", nullptr));
        contiBufferValueLabel->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        contiSpaceLabel->setText(QCoreApplication::translate("MainWindow", "\347\274\223\345\206\262\345\211\251\344\275\231\347\251\272\351\227\264", nullptr));
        contiSpaceValueLabel->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        contiHostQueueLabel->setText(QCoreApplication::translate("MainWindow", "\344\270\212\344\275\215\346\234\272\351\230\237\345\210\227", nullptr));
        contiHostQueueValueLabel->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        contiPlanningAlignmentLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\350\247\204\345\210\222\345\221\250\346\234\237", nullptr));
        contiPlanningAlignmentValueLabel->setText(QCoreApplication::translate("MainWindow", "\345\276\205\345\210\235\345\247\213\345\214\226", nullptr));
        contiExpectedPlanTimeLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\237\346\234\233 / \345\215\241\344\276\247\350\256\241\345\210\222", nullptr));
        contiExpectedPlanTimeValueLabel->setText(QCoreApplication::translate("MainWindow", "0.0 / 0.0 ms", nullptr));
        contiPhaseErrorLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\270\344\275\215\350\257\257\345\267\256", nullptr));
        contiPhaseErrorValueLabel->setText(QCoreApplication::translate("MainWindow", "0.0 ms", nullptr));
        contiBufferTimeLabel->setText(QCoreApplication::translate("MainWindow", "\350\247\204\345\210\222\347\274\223\345\206\262", nullptr));
        contiBufferTimeValueLabel->setText(QCoreApplication::translate("MainWindow", "0.0 ms", nullptr));
        contiRatioApiAgeLabel->setText(QCoreApplication::translate("MainWindow", "\345\200\215\347\216\207 API \350\267\235\344\273\212", nullptr));
        contiRatioApiAgeValueLabel->setText(QCoreApplication::translate("MainWindow", "-- ms", nullptr));
        contiRatioDiagLabel->setText(QCoreApplication::translate("MainWindow", "\345\200\215\347\216\207 ref/phase/buffer/cmd/applied", nullptr));
        contiRatioDiagValueLabel->setText(QCoreApplication::translate("MainWindow", "0.000 / 0.000 / 0.000 / 0.000 / 0.000", nullptr));
        testGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\265\213\350\257\225\350\275\250\350\277\271\345\217\202\346\225\260", nullptr));
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
        cardUnitDefinitionLabel->setText(QCoreApplication::translate("MainWindow", "\346\235\277\345\215\241 unit \345\256\232\344\271\211", nullptr));
        cardUnitDefinitionCombo->setItemText(0, QCoreApplication::translate("MainWindow", "500.622 pulse/unit\357\274\2101 unit = 1\302\260\357\274\211", nullptr));
        cardUnitDefinitionCombo->setItemText(1, QCoreApplication::translate("MainWindow", "50.0622 pulse/unit\357\274\2101 unit = 0.1\302\260\357\274\211", nullptr));
        cardUnitDefinitionCombo->setItemText(2, QCoreApplication::translate("MainWindow", "5.00622 pulse/unit\357\274\2101 unit = 0.01\302\260\357\274\211", nullptr));

        trajectoryPointModeLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\344\272\247\347\202\271\346\226\271\345\274\217", nullptr));
        trajectoryPointModeCombo->setItemText(0, QCoreApplication::translate("MainWindow", "\344\272\224\346\254\241\345\244\232\351\241\271\345\274\217\357\274\210\346\255\243\345\274\217\350\275\250\350\277\271\357\274\211", nullptr));
        trajectoryPointModeCombo->setItemText(1, QCoreApplication::translate("MainWindow", "\347\255\211\351\227\264\350\267\235\347\233\264\347\272\277\357\274\210\345\211\215\347\236\273\345\257\271\347\205\247\357\274\211", nullptr));

        contiGroup->setTitle(QCoreApplication::translate("MainWindow", "\350\277\236\347\273\255\346\217\222\350\241\245\345\217\202\346\225\260", nullptr));
        maxVelocityLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\200\345\244\247\347\237\242\351\207\217\351\200\237\345\272\246", nullptr));
        maxVelocitySpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        accelerationLabel->setText(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\346\227\266\351\227\264", nullptr));
        accelerationSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        decelerationLabel->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\346\227\266\351\227\264", nullptr));
        decelerationSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        sCurveLabel->setText(QCoreApplication::translate("MainWindow", "S \346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
        sCurveSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        speedRatioLabel->setText(QCoreApplication::translate("MainWindow", "\345\200\215\347\216\207\344\270\212\351\231\220", nullptr));
        lookaheadLabel->setText(QCoreApplication::translate("MainWindow", "\345\260\217\347\272\277\346\256\265\345\211\215\347\236\273", nullptr));
        lookaheadCheck->setText(QCoreApplication::translate("MainWindow", "\345\220\257\347\224\250", nullptr));
        pathErrorLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\345\205\201\350\256\270\350\257\257\345\267\256", nullptr));
        pathErrorSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        preloadSegmentsLabel->setText(QCoreApplication::translate("MainWindow", "\345\220\257\345\212\250\351\242\204\345\216\213\346\227\266\351\227\264", nullptr));
        preloadSegmentsSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
#if QT_CONFIG(tooltip)
        preloadAllTrajectoryCheck->setToolTip(QCoreApplication::translate("MainWindow", "\345\220\257\345\212\250\345\211\215\345\260\206\345\205\250\351\203\250\346\234\211\346\225\210\350\275\250\350\277\271\346\256\265\345\216\213\345\205\245\346\216\247\345\210\266\345\215\241\343\200\202\344\273\205\351\200\202\347\224\250\344\272\216\346\234\252\350\266\205\350\277\207\345\215\241\344\276\247\345\217\257\347\224\250\346\256\265\345\256\271\351\207\217\347\232\204\347\237\255\350\275\250\350\277\271\357\274\233\350\277\220\350\241\214\344\270\255\344\270\215\345\206\215\344\272\247\347\202\271\346\210\226\350\241\245\346\256\265\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        preloadAllTrajectoryCheck->setText(QCoreApplication::translate("MainWindow", "\344\270\200\346\254\241\346\200\247\351\242\204\350\243\205\345\205\250\351\203\250\350\275\250\350\277\271\357\274\210\345\257\271\347\205\247\357\274\211", nullptr));
        targetBufferLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\256\346\240\207\347\274\223\345\206\262\346\227\266\351\227\264", nullptr));
        targetBufferSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        lowBufferLabel->setText(QCoreApplication::translate("MainWindow", "\344\275\216\346\260\264\344\275\215\346\227\266\351\227\264", nullptr));
        lowBufferSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        timeSyncEnableLabel->setText(QCoreApplication::translate("MainWindow", "\346\227\266\351\227\264\345\220\214\346\255\245\345\200\215\347\216\207\346\216\247\345\210\266", nullptr));
        timeSyncEnableCheck->setText(QCoreApplication::translate("MainWindow", "\345\220\257\347\224\250", nullptr));
        criticalBufferLabel->setText(QCoreApplication::translate("MainWindow", "\345\215\261\351\231\251\346\260\264\344\275\215\346\227\266\351\227\264", nullptr));
        criticalBufferSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        ratioMinLabel->setText(QCoreApplication::translate("MainWindow", "\345\200\215\347\216\207\344\270\213\351\231\220", nullptr));
        phaseGainLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\270\344\275\215\345\242\236\347\233\212", nullptr));
        ratioDeadbandLabel->setText(QCoreApplication::translate("MainWindow", "\345\200\215\347\216\207\345\217\230\345\214\226\346\255\273\345\214\272", nullptr));
        ratioStepLabel->setText(QCoreApplication::translate("MainWindow", "\345\215\225\346\254\241\345\200\215\347\216\207\345\217\230\345\214\226\344\270\212\351\231\220", nullptr));
        ratioPeriodLabel->setText(QCoreApplication::translate("MainWindow", "\345\200\215\347\216\207\346\216\247\345\210\266\345\221\250\346\234\237", nullptr));
        ratioPeriodSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        markOffsetLabel->setText(QCoreApplication::translate("MainWindow", "mark \346\240\241\345\207\206\345\201\217\347\247\273", nullptr));
        bufferGainLabel->setText(QCoreApplication::translate("MainWindow", "\347\274\223\345\206\262\346\260\264\344\275\215\345\242\236\347\233\212", nullptr));
        executionDelayLabel->setText(QCoreApplication::translate("MainWindow", "\345\233\272\345\256\232\346\211\247\350\241\214\345\273\266\350\277\237", nullptr));
        executionDelaySpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        phaseDeadbandLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\270\344\275\215\350\257\257\345\267\256\346\255\273\345\214\272", nullptr));
        phaseDeadbandSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        ratioApiIntervalLabel->setText(QCoreApplication::translate("MainWindow", "\346\231\256\351\200\232\345\200\215\347\216\207 API \346\234\200\345\260\217\351\227\264\351\232\224", nullptr));
        ratioApiIntervalSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        ratioSafetyApiIntervalLabel->setText(QCoreApplication::translate("MainWindow", "\345\256\211\345\205\250\345\200\215\347\216\207 API \346\234\200\345\260\217\351\227\264\351\232\224", nullptr));
        ratioSafetyApiIntervalSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        startButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\350\277\236\347\273\255\346\217\222\350\241\245", nullptr));
        stopButton->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\201\234\346\255\242", nullptr));
        contiTrajectoryGroup->setTitle(QCoreApplication::translate("MainWindow", "\344\270\273\345\212\250\350\275\264\350\275\250\350\277\271\345\257\271\346\257\224\357\274\210\344\275\216\351\242\221\346\230\276\347\244\272\357\274\211", nullptr));
#if QT_CONFIG(tooltip)
        contiTrajectoryChartView->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\273\232\350\275\256\344\273\245\345\205\211\346\240\207\344\275\215\347\275\256\344\270\272\344\270\255\345\277\203\347\274\251\346\224\276\346\250\252\347\272\265\345\235\220\346\240\207\357\274\233\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        tabWidget->setTabText(tabWidget->indexOf(contiTestTab), QCoreApplication::translate("MainWindow", "\350\277\236\347\273\255\346\217\222\350\241\245\346\265\213\350\257\225", nullptr));
        baseConfigGroup->setTitle(QCoreApplication::translate("MainWindow", "E5000 \345\237\272\347\241\200\351\205\215\347\275\256", nullptr));
        axisRangeLabel->setText(QCoreApplication::translate("MainWindow", "\345\237\272\347\241\200\351\205\215\347\275\256\350\275\264\350\214\203\345\233\264", nullptr));
        axisRangeValueLabel->setText(QCoreApplication::translate("MainWindow", "\345\275\223\345\211\215\347\224\261\342\200\234\344\270\273\345\212\250\350\275\264\342\200\235\345\222\214\342\200\234\344\277\235\346\214\201/\347\254\254\344\272\214\350\275\264\342\200\235\351\200\211\346\213\251", nullptr));
        equivLabel->setText(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
        equivValueLabel->setText(QCoreApplication::translate("MainWindow", "500.622 pulse/unit\357\274\2101 unit = 1\302\260\357\274\233180224 pulse/rev\357\274\211", nullptr));
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
#if QT_CONFIG(tooltip)
        positionChartView->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\273\232\350\275\256\344\273\245\345\205\211\346\240\207\344\275\215\347\275\256\344\270\272\344\270\255\345\277\203\347\274\251\346\224\276\346\250\252\347\272\265\345\235\220\346\240\207\357\274\233\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        followingErrorChartView->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\273\232\350\275\256\344\273\245\345\205\211\346\240\207\344\275\215\347\275\256\344\270\272\344\270\255\345\277\203\347\274\251\346\224\276\346\250\252\347\272\265\345\235\220\346\240\207\357\274\233\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        telemetryHintLabel->setText(QCoreApplication::translate("MainWindow", "\347\202\271\345\207\273\342\200\234\345\274\200\345\247\213\350\256\260\345\275\225\342\200\235\345\220\216\357\274\214\346\233\262\347\272\277\344\273\245 20 Hz \346\230\276\347\244\272\346\234\200\346\226\260 Trace \345\277\253\347\205\247\357\274\233\347\202\271\345\207\273\342\200\234\345\201\234\346\255\242\350\256\260\345\275\225\345\271\266\345\206\231\345\205\245\346\226\207\344\273\266\342\200\235\345\220\216\345\201\234\346\255\242\350\277\275\345\212\240\343\200\202\345\216\237\345\247\213 1 ms Trace \345\270\247\347\224\261\347\213\254\347\253\213\345\206\231\347\233\230\347\272\277\347\250\213\344\277\235\345\255\230\343\200\202\347\273\230\345\233\276\345\214\272\345\206\205\346\273\232\345\212\250\346\273\232\350\275\256\345\217\257\347\274\251\346\224\276\357\274\214\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(telemetryTab), QCoreApplication::translate("MainWindow", "\346\225\260\346\215\256\350\256\260\345\275\225\344\270\216\346\233\262\347\272\277", nullptr));
        velocityControlParameterGroup->setTitle(QCoreApplication::translate("MainWindow", "\345\215\225\350\275\264\351\200\237\345\272\246\346\250\241\345\274\217\344\275\215\347\275\256\351\227\255\347\216\257\345\217\202\346\225\260", nullptr));
        velocityTrajectoryGroup->setTitle(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\350\256\276\347\275\256", nullptr));
        velocityAxisLabel->setText(QCoreApplication::translate("MainWindow", "\346\265\213\350\257\225\350\275\264", nullptr));
        velocityAxisCombo->setItemText(0, QCoreApplication::translate("MainWindow", "0", nullptr));
        velocityAxisCombo->setItemText(1, QCoreApplication::translate("MainWindow", "1", nullptr));
        velocityAxisCombo->setItemText(2, QCoreApplication::translate("MainWindow", "2", nullptr));
        velocityAxisCombo->setItemText(3, QCoreApplication::translate("MainWindow", "3", nullptr));
        velocityAxisCombo->setItemText(4, QCoreApplication::translate("MainWindow", "4", nullptr));
        velocityAxisCombo->setItemText(5, QCoreApplication::translate("MainWindow", "5", nullptr));
        velocityAxisCombo->setItemText(6, QCoreApplication::translate("MainWindow", "6", nullptr));
        velocityAxisCombo->setItemText(7, QCoreApplication::translate("MainWindow", "7", nullptr));

        velocityDeltaLabel->setText(QCoreApplication::translate("MainWindow", "\347\233\270\345\257\271\344\275\215\347\247\273", nullptr));
        velocityDeltaSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        velocityDurationLabel->setText(QCoreApplication::translate("MainWindow", "\350\275\250\350\277\271\346\227\266\351\227\264", nullptr));
        velocityDurationSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        velocityControlPeriodLabel->setText(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\345\221\250\346\234\237", nullptr));
        velocityControlPeriodSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        velocityClosedLoopGroup->setTitle(QCoreApplication::translate("MainWindow", "\351\227\255\347\216\257\346\216\247\345\210\266", nullptr));
        velocityFeedforwardCheck->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246\345\211\215\351\246\210", nullptr));
        velocityPidEnableCheck->setText(QCoreApplication::translate("MainWindow", "\345\220\257\347\224\250PID", nullptr));
        velocityKpLabel->setText(QCoreApplication::translate("MainWindow", "Kp (1/s)", nullptr));
        velocityKiLabel->setText(QCoreApplication::translate("MainWindow", "Ki (1/s\302\262)", nullptr));
        velocityKdLabel->setText(QCoreApplication::translate("MainWindow", "Kd", nullptr));
        velocityIntegralLimitLabel->setText(QCoreApplication::translate("MainWindow", "\347\247\257\345\210\206\347\212\266\346\200\201\344\270\212\351\231\220", nullptr));
        velocityIntegralLimitSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260\302\267s", nullptr));
        velocityMaxCorrectionLabel->setText(QCoreApplication::translate("MainWindow", "PID\344\277\256\346\255\243\344\270\212\351\231\220", nullptr));
        velocityMaxCorrectionSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        velocityLimitGroup->setTitle(QCoreApplication::translate("MainWindow", "\350\277\220\345\212\250\351\231\220\345\210\266", nullptr));
        velocityMaxSpeedLabel->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246\344\270\212\351\231\220", nullptr));
        velocityMaxSpeedSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        velocityMaxAccelerationLabel->setText(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246\344\270\212\351\231\220", nullptr));
        velocityMaxAccelerationSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s\302\262", nullptr));
        velocityChangeTimeLabel->setText(QCoreApplication::translate("MainWindow", "\345\234\250\347\272\277\345\217\230\351\200\237\346\227\266\351\227\264", nullptr));
        velocityChangeTimeSpin->setSuffix(QCoreApplication::translate("MainWindow", " s", nullptr));
        velocityStartThresholdLabel->setText(QCoreApplication::translate("MainWindow", "\345\220\257\345\212\250\351\200\237\345\272\246\351\230\210\345\200\274", nullptr));
        velocityStartThresholdSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        velocityCriterionGroup->setTitle(QCoreApplication::translate("MainWindow", "\345\210\244\345\256\232\344\277\235\346\212\244", nullptr));
        velocityPositionToleranceLabel->setText(QCoreApplication::translate("MainWindow", "\344\275\215\347\275\256\345\256\271\345\267\256", nullptr));
        velocityPositionToleranceSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        velocitySpeedToleranceLabel->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246\345\256\271\345\267\256", nullptr));
        velocitySpeedToleranceSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260/s", nullptr));
        velocityStableDwellLabel->setText(QCoreApplication::translate("MainWindow", "\347\250\263\346\200\201\347\241\256\350\256\244", nullptr));
        velocityStableDwellSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        velocityFinishTimeoutLabel->setText(QCoreApplication::translate("MainWindow", "\347\273\210\347\202\271\350\266\205\346\227\266", nullptr));
        velocityFinishTimeoutSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        velocityMaxFollowingErrorLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\200\345\244\247\350\267\237\351\232\217\350\257\257\345\267\256", nullptr));
        velocityMaxFollowingErrorSpin->setSuffix(QCoreApplication::translate("MainWindow", " \302\260", nullptr));
        velocityTraceTimeoutLabel->setText(QCoreApplication::translate("MainWindow", "Trace\350\266\205\346\227\266", nullptr));
        velocityTraceTimeoutSpin->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        velocityControlOperationGroup->setTitle(QCoreApplication::translate("MainWindow", "\346\223\215\344\275\234\344\270\216\347\212\266\346\200\201", nullptr));
        velocityEnableAxisButton->setText(QCoreApplication::translate("MainWindow", "\344\275\277\350\203\275\346\265\213\350\257\225\350\275\264", nullptr));
        velocityDisableAxisButton->setText(QCoreApplication::translate("MainWindow", "\345\244\261\350\203\275\346\265\213\350\257\225\350\275\264", nullptr));
        velocityResetButton->setText(QCoreApplication::translate("MainWindow", "\351\207\215\347\275\256\346\216\247\345\210\266\345\231\250", nullptr));
        velocityStartButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\345\215\225\350\275\264\351\227\255\347\216\257", nullptr));
        velocityStopButton->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\201\234\346\255\242", nullptr));
        velocityEmergencyStopButton->setText(QCoreApplication::translate("MainWindow", "\347\253\213\345\215\263\345\201\234\346\255\242", nullptr));
        velocityClearCurvesButton->setText(QCoreApplication::translate("MainWindow", "\346\270\205\351\231\244\346\234\254\346\254\241\346\233\262\347\272\277", nullptr));
        velocityStateLabel->setText(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\347\212\266\346\200\201", nullptr));
        velocityStateValueLabel->setText(QCoreApplication::translate("MainWindow", "\346\234\252\350\277\220\350\241\214", nullptr));
        velocityTimingLabel->setText(QCoreApplication::translate("MainWindow", "\346\227\266\351\227\264 / dt / \346\234\200\345\244\247\346\212\226\345\212\250", nullptr));
        velocityTimingValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0 / 0", nullptr));
        velocityPositionStatusLabel->setText(QCoreApplication::translate("MainWindow", "\344\275\215\347\275\256 ref / card / actual / error", nullptr));
        velocityPositionStatusValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0 / 0 / 0 \302\260", nullptr));
        velocitySpeedStatusLabel->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246 ref / cmd / card / actual", nullptr));
        velocitySpeedStatusValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0 / 0 / 0 \302\260/s", nullptr));
        velocityPidStatusLabel->setText(QCoreApplication::translate("MainWindow", "\345\211\215\351\246\210 / P / I / D / \351\231\220\345\271\205\347\212\266\346\200\201", nullptr));
        velocityPidStatusValueLabel->setText(QCoreApplication::translate("MainWindow", "0 / 0 / 0 / 0 / --", nullptr));
#if QT_CONFIG(tooltip)
        velocityPositionChartView->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\273\232\350\275\256\344\273\245\345\205\211\346\240\207\344\275\215\347\275\256\344\270\272\344\270\255\345\277\203\347\274\251\346\224\276\346\250\252\347\272\265\345\235\220\346\240\207\357\274\233\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        velocityErrorChartView->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\273\232\350\275\256\344\273\245\345\205\211\346\240\207\344\275\215\347\275\256\344\270\272\344\270\255\345\277\203\347\274\251\346\224\276\346\250\252\347\272\265\345\235\220\346\240\207\357\274\233\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        velocitySpeedChartView->setToolTip(QCoreApplication::translate("MainWindow", "\351\274\240\346\240\207\346\273\232\350\275\256\344\273\245\345\205\211\346\240\207\344\275\215\347\275\256\344\270\272\344\270\255\345\277\203\347\274\251\346\224\276\346\250\252\347\272\265\345\235\220\346\240\207\357\274\233\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202", nullptr));
#endif // QT_CONFIG(tooltip)
        velocityControlHintLabel->setText(QCoreApplication::translate("MainWindow", "\346\216\247\345\210\266\346\225\260\346\215\256\346\214\211\346\216\247\345\210\266\345\221\250\346\234\237\344\272\247\347\224\237\357\274\233UI \346\257\217 50 ms \347\273\230\345\210\266\346\234\200\346\226\260\345\277\253\347\205\247\343\200\202\350\277\220\350\241\214\347\273\223\346\235\237\346\210\226\345\201\234\346\255\242\345\220\216\346\233\262\347\272\277\347\253\213\345\215\263\345\201\234\346\255\242\350\277\275\345\212\240\343\200\202\347\273\230\345\233\276\345\214\272\345\206\205\346\273\232\345\212\250\346\273\232\350\275\256\345\217\257\347\274\251\346\224\276\357\274\214\345\217\214\345\207\273\346\201\242\345\244\215\350\207\252\345\212\250\351\207\217\347\250\213\343\200\202\351\246\226\346\254\241\350\257\225\351\252\214\350\257\267\345\205\210\345\205\263\351\227\255 PID\357\274\214\344\273\205\347\224\250\344\275\216\351\200\237\345\211\215\351\246\210\346\240\270\345\257\271 Trace type 3/4 \347\232\204\345\215\225\344\275\215\345\222\214\346\226\271\345\220\221\343\200\202", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(velocityControlTab), QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246\351\227\255\347\216\257\346\265\213\350\257\225", nullptr));
        extensionHintLabel->setText(QCoreApplication::translate("MainWindow", "\346\255\244\351\241\265\351\235\242\351\242\204\347\225\231\347\273\231\345\220\216\347\273\255\347\232\204 8 \350\275\264 CDPR\343\200\201\345\212\250\345\212\233\345\255\246\346\261\202\350\247\243\343\200\201\345\212\233\344\274\240\346\204\237\345\231\250\345\222\214\347\274\223\345\206\262\346\260\264\344\275\215\350\207\252\345\212\250\350\260\203\351\200\237\347\255\211\345\212\237\350\203\275\343\200\202", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(extensionTab), QCoreApplication::translate("MainWindow", "\346\211\251\345\261\225\345\212\237\350\203\275\357\274\210\351\242\204\347\225\231\357\274\211", nullptr));
        logGroup->setTitle(QCoreApplication::translate("MainWindow", "\350\277\220\350\241\214\346\227\245\345\277\227", nullptr));
        copyLogButton->setText(QCoreApplication::translate("MainWindow", "\345\244\215\345\210\266\346\227\245\345\277\227", nullptr));
        clearLogButton->setText(QCoreApplication::translate("MainWindow", "\346\270\205\351\231\244\346\227\245\345\277\227", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
