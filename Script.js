
(function (global) {
  "use strict";

  // ФИЗИЧЕСКИЕ КОНСТАНТЫ

  const MU = 3.986004418e14;
  const EARTH_RADIUS = 6_371_000;
  const G0 = 9.80665;


  // РАСЧЁТ ОРБИТАЛЬНЫХ ЭЛЕМЕНТОВ

  function orbitalElements(state) {

    const [x, y, vx, vy] = state;

    const radius = Math.hypot(x, y);
    const speed2 = vx * vx + vy * vy;

    const energy = speed2 / 2 - MU / radius;
    const radialVelocity = x * vx + y * vy;

    const ex = (
      (speed2 - MU / radius) * x -
      radialVelocity * vx
    ) / MU;

    const ey = (
      (speed2 - MU / radius) * y -
      radialVelocity * vy
    ) / MU;

    const eccentricity = Math.hypot(ex, ey);

    const semiMajor = energy < 0
      ? -MU / (2 * energy)
      : Infinity;

    return {

      altitudeKm: (radius - EARTH_RADIUS) / 1000,

      meanAltitudeKm: Number.isFinite(semiMajor)
        ? (semiMajor - EARTH_RADIUS) / 1000
        : Infinity,

      speedKms: Math.sqrt(speed2) / 1000,

      eccentricity,

      periapsisKm: Number.isFinite(semiMajor)
        ? (semiMajor * (1 - eccentricity) - EARTH_RADIUS) / 1000
        : -Infinity,

      apoapsisKm: Number.isFinite(semiMajor)
        ? (semiMajor * (1 + eccentricity) - EARTH_RADIUS) / 1000
        : Infinity

    };
  }


  // РАСЧЁТ УСКОРЕНИЙ

  function derivative(state, mass, thrustN, angle, thrustFactor) {

    const [x, y, vx, vy] = state;

    const radius = Math.hypot(x, y);

    const gravity = -MU / (radius * radius * radius);

    const radialX = x / radius;
    const radialY = y / radius;

    const tangentialX = -radialY;
    const tangentialY = radialX;

    const thrustAcceleration = thrustFactor * thrustN / mass;

    const directionX =
      Math.cos(angle) * tangentialX +
      Math.sin(angle) * radialX;

    const directionY =
      Math.cos(angle) * tangentialY +
      Math.sin(angle) * radialY;

    return [

      vx,
      vy,

      gravity * x + thrustAcceleration * directionX,
      gravity * y + thrustAcceleration * directionY

    ];
  }


  // МЕТОД РУНГЕ — КУТТЫ 4-ГО ПОРЯДКА

  function rk4Step(state, dt, mass, thrustN, angle, thrustFactor) {

    const add = (base, delta, factor) =>
      base.map((value, index) =>
        value + delta[index] * factor
      );

    const k1 = derivative(
      state, mass, thrustN, angle, thrustFactor
    );

    const k2 = derivative(
      add(state, k1, dt / 2),
      mass, thrustN, angle, thrustFactor
    );

    const k3 = derivative(
      add(state, k2, dt / 2),
      mass, thrustN, angle, thrustFactor
    );

    const k4 = derivative(
      add(state, k3, dt),
      mass, thrustN, angle, thrustFactor
    );

    return state.map((value, index) =>
      value + dt * (
        k1[index] +
        2 * k2[index] +
        2 * k3[index] +
        k4[index]
      ) / 6
    );
  }


  // ОСНОВНАЯ ФУНКЦИЯ МОДЕЛИРОВАНИЯ

  function simulateMission(config) {

    const initialRadius =
      EARTH_RADIUS + config.initialAltitude * 1000;

    let state = [
      initialRadius,
      0,
      0,
      Math.sqrt(MU / initialRadius)
    ];

    let fuel = config.propellant;
    let time = 0;

    let reason = "time";
    let targetCrossed = false;
    let targetTime = null;
    let fuelAtTarget = null;

    const thrustN = config.thrust / 1000;

    const angle = config.thrustAngle * Math.PI / 180;

    const massFlow = thrustN / (config.isp * G0);

    const totalTime = config.durationDays * 86400;

    const steps = Math.min(
      120000,
      Math.max(2500, Math.ceil(totalTime / 10))
    );

    const dt = totalTime / steps;

    const captureEvery = Math.max(
      1,
      Math.floor(steps / 1500)
    );

    const records = [];


    // СОХРАНЕНИЕ СОСТОЯНИЯ АППАРАТА

    function capture(elements) {

      const radius = Math.hypot(state[0], state[1]);

      const radialX = state[0] / radius;
      const radialY = state[1] / radius;

      records.push({

        t: time,

        x: state[0],
        y: state[1],

        fuel,

        directionX:
          Math.cos(angle) * -radialY +
          Math.sin(angle) * radialX,

        directionY:
          Math.cos(angle) * radialX +
          Math.sin(angle) * radialY,

        ...elements

      });
    }


    capture(orbitalElements(state));


    // ЦИКЛ МОДЕЛИРОВАНИЯ

    for (let step = 1; step <= steps; step += 1) {

      const requestedFuel = massFlow * dt;

      const burned = Math.min(fuel, requestedFuel);

      const thrustFactor = requestedFuel > 0
        ? burned / requestedFuel
        : 0;

      const averageMass =
        config.dryMass + fuel - burned / 2;

      state = rk4Step(
        state,
        dt,
        averageMass,
        thrustN,
        angle,
        thrustFactor
      );

      fuel = Math.max(0, fuel - burned);

      time += dt;

      const elements = orbitalElements(state);


      // ПРОВЕРКА ДОСТИЖЕНИЯ ЦЕЛЕВОЙ ОРБИТЫ

      if (
        !targetCrossed &&
        elements.meanAltitudeKm >= config.targetAltitude
      ) {

        targetCrossed = true;
        targetTime = time;
        fuelAtTarget = fuel;

      }


      // ПРОВЕРКА ОПАСНОГО СНИЖЕНИЯ

      if (elements.altitudeKm < 80) {

        reason = "reentry";
        capture(elements);
        break;

      }


      // ПРОВЕРКА ПОТЕРИ СВЯЗАННОЙ ОРБИТЫ

      if (
        !Number.isFinite(elements.meanAltitudeKm) ||
        Math.hypot(state[0], state[1]) > 1.2e8
      ) {

        reason = "escape";
        capture(elements);
        break;

      }


      // АВТОСТОП

      if (config.autoStop && targetCrossed) {

        reason = "target";
        capture(elements);
        break;

      }


      // СОХРАНЕНИЕ ПРОМЕЖУТОЧНЫХ ДАННЫХ

      if (step % captureEvery === 0 || step === steps) {

        capture(elements);

      }

    }


    // ИТОГОВЫЕ ПАРАМЕТРЫ

    const finalElements = orbitalElements(state);

    if (records[records.length - 1].t !== time) {
      capture(finalElements);
    }

    const toleranceKm = Math.max(
      10,
      Math.abs(
        config.targetAltitude - config.initialAltitude
      ) * 0.03
    );


    // КРИТЕРИИ УСПЕШНОСТИ

    const criteria = {

      target:
        Math.abs(
          finalElements.meanAltitudeKm -
          config.targetAltitude
        ) <= toleranceKm,

      circular:
        finalElements.eccentricity <= 0.05,

      safe:
        finalElements.periapsisKm >= 120,

      fuel:
        fuel > 0.01

    };


    return {

      config,
      records,
      reason,

      targetCrossed,
      targetTime,
      fuelAtTarget,

      final: records[records.length - 1],

      toleranceKm,

      consumed: config.propellant - fuel,

      criteria,

      success: Object.values(criteria).every(Boolean)

    };

  }


  // ЭКСПОРТ ДЛЯ ТЕСТИРОВАНИЯ В NODE.JS

  if (typeof module !== "undefined" && module.exports) {

    module.exports = {
      simulateMission,
      orbitalElements,
      constants: {
        MU,
        EARTH_RADIUS,
        G0
      }
    };

  }

  if (typeof document === "undefined") return;


  // ПОЛУЧЕНИЕ ЭЛЕМЕНТОВ HTML

  const root = document.getElementById("spiral-transfer-app");

  if (!root) return;

  const $ = (id) => root.querySelector(`#${id}`);

  const orbitCanvas = $("orbitCanvas");
  const chartCanvas = $("altitudeChart");

  const form = $("missionForm");


  // СОСТОЯНИЕ ПРИЛОЖЕНИЯ

  let result = null;

  let framePosition = 0;
  let animationId = null;
  let lastTimestamp = null;

  let paused = false;

  const prefersReducedMotion =
    global.matchMedia &&
    global.matchMedia("(prefers-reduced-motion: reduce)").matches;

  const numberFormat = new Intl.NumberFormat(
    "ru-RU",
    { maximumFractionDigits: 2 }
  );


  // ПОЛУЧЕНИЕ ПАРАМЕТРОВ ИЗ ФОРМЫ

  function configFromForm() {

    return {

      initialAltitude: Number($("initialAltitude").value),

      targetAltitude: Number($("targetAltitude").value),

      dryMass: Number($("dryMass").value),

      propellant: Number($("propellant").value),

      thrust: Number($("thrust").value),

      isp: Number($("isp").value),

      thrustAngle: Number($("thrustAngle").value),

      durationDays: Number($("durationDays").value),

      autoStop: $("autoStop").checked

    };

  }


  // ПРОВЕРКА ВВЕДЁННЫХ ДАННЫХ

  function validate(config) {

    if (
      Object.entries(config).some(
        ([key, value]) =>
          key !== "autoStop" &&
          !Number.isFinite(value)
      )
    ) {
      return "Заполните все числовые поля.";
    }

    if (!form.checkValidity()) {

      return "Проверьте числовые поля: значения должны соответствовать указанным диапазонам и шагам.";

    }

    if (config.targetAltitude <= config.initialAltitude) {

      return "Целевая высота должна быть выше начальной.";

    }

    if (config.initialAltitude < 160) {

      return "Начальная орбита ниже 160 км быстро разрушается атмосферой.";

    }

    if (
      config.dryMass <= 0 ||
      config.propellant <= 0 ||
      config.thrust <= 0 ||
      config.isp <= 0 ||
      config.durationDays <= 0
    ) {

      return "Масса, тяга, импульс и время должны быть положительными.";

    }

    return "";

  }


  // АДАПТАЦИЯ CANVAS ПОД РАЗМЕР ЭКРАНА

  function fitCanvas(canvas) {

    const rect = canvas.getBoundingClientRect();

    const ratio = Math.min(global.devicePixelRatio || 1, 2);

    const width = Math.max(1, Math.round(rect.width));

    const height = Math.max(1, Math.round(rect.height));

    if (
      canvas.width !== width * ratio ||
      canvas.height !== height * ratio
    ) {

      canvas.width = width * ratio;
      canvas.height = height * ratio;

    }

    const context = canvas.getContext("2d");

    context.setTransform(ratio, 0, 0, ratio, 0, 0);

    return {
      context,
      width,
      height
    };

  }


  // ОТРИСОВКА ОРБИТЫ

  function drawOrbit(index) {

    const {
      context: ctx,
      width,
      height
    } = fitCanvas(orbitCanvas);

    ctx.clearRect(0, 0, width, height);

    const records = result ? result.records : [];

    const config = result
      ? result.config
      : configFromForm();

    const current = records[index] || {

      x: EARTH_RADIUS + config.initialAltitude * 1000,

      y: 0,

      directionX: 0,

      directionY: 1,

      altitudeKm: config.initialAltitude,

      meanAltitudeKm: config.initialAltitude,

      speedKms:
        Math.sqrt(
          MU /
          (EARTH_RADIUS + config.initialAltitude * 1000)
        ) / 1000,

      fuel: config.propellant,

      t: 0

    };


    const maxRadius = Math.max(
      EARTH_RADIUS + config.targetAltitude * 1000,
      ...records.map(
        (point) => Math.hypot(point.x, point.y)
      )
    ) * 1.08;

    const scale =
      Math.min(width, height) * 0.43 / maxRadius;

    const cx = width / 2;
    const cy = height / 2;


    // ЗВЁЗДНЫЙ ФОН

    for (let i = 0; i < 55; i += 1) {

      const sx = ((i * 83) % 997) / 997 * width;

      const sy = ((i * 191) % 991) / 991 * height;

      ctx.fillStyle =
        `rgba(255,255,255,${0.18 + (i % 5) * 0.07})`;

      ctx.fillRect(
        sx,
        sy,
        i % 9 === 0 ? 2 : 1,
        i % 9 === 0 ? 2 : 1
      );

    }


    // ЦЕЛЕВАЯ ОРБИТА

    const targetRadius =
      (EARTH_RADIUS + config.targetAltitude * 1000) * scale;

    ctx.setLineDash([7, 8]);

    ctx.strokeStyle = "#fb923c";
    ctx.lineWidth = 1.5;

    ctx.beginPath();

    ctx.arc(
      cx,
      cy,
      targetRadius,
      0,
      Math.PI * 2
    );

    ctx.stroke();

    ctx.setLineDash([]);


    // ТРАЕКТОРИЯ

    if (records.length > 1) {

      ctx.beginPath();

      for (let i = 0; i <= index; i += 1) {

        const px = cx + records[i].x * scale;

        const py = cy - records[i].y * scale;

        if (i === 0) {
          ctx.moveTo(px, py);
        } else {
          ctx.lineTo(px, py);
        }

      }

      ctx.strokeStyle = "#93c5fd";
      ctx.lineWidth = 2;

      ctx.stroke();

    }


    // ЗЕМЛЯ

    const earthRadiusPx = EARTH_RADIUS * scale;

    const atmosphere = ctx.createRadialGradient(
      cx,
      cy,
      earthRadiusPx * .78,
      cx,
      cy,
      earthRadiusPx * 1.12
    );

    atmosphere.addColorStop(0, "#2563eb");

    atmosphere.addColorStop(.83, "#1d4ed8");

    atmosphere.addColorStop(1, "rgba(96,165,250,0)");

    ctx.fillStyle = atmosphere;

    ctx.beginPath();

    ctx.arc(
      cx,
      cy,
      earthRadiusPx * 1.12,
      0,
      Math.PI * 2
    );

    ctx.fill();


    ctx.fillStyle = "#1e40af";

    ctx.beginPath();

    ctx.arc(
      cx,
      cy,
      earthRadiusPx,
      0,
      Math.PI * 2
    );

    ctx.fill();


    // УСЛОВНЫЕ КОНТИНЕНТЫ

    ctx.fillStyle = "rgba(34,197,94,.48)";

    ctx.beginPath();

    ctx.ellipse(
      cx - earthRadiusPx * .2,
      cy - earthRadiusPx * .1,
      earthRadiusPx * .34,
      earthRadiusPx * .18,
      -.4,
      0,
      Math.PI * 2
    );

    ctx.fill();


    ctx.beginPath();

    ctx.ellipse(
      cx + earthRadiusPx * .25,
      cy + earthRadiusPx * .24,
      earthRadiusPx * .23,
      earthRadiusPx * .13,
      .5,
      0,
      Math.PI * 2
    );

    ctx.fill();


    // КОСМИЧЕСКИЙ АППАРАТ

    const craftX = cx + current.x * scale;

    const craftY = cy - current.y * scale;

    ctx.fillStyle = "#fff";

    ctx.strokeStyle = "#818cf8";

    ctx.lineWidth = 3;

    ctx.beginPath();

    ctx.arc(
      craftX,
      craftY,
      6,
      0,
      Math.PI * 2
    );

    ctx.fill();

    ctx.stroke();


    // ВЕКТОР ТЯГИ

    const arrowLength = 42;

    const arrowX =
      craftX + current.directionX * arrowLength;

    const arrowY =
      craftY - current.directionY * arrowLength;

    ctx.strokeStyle = "#f97316";

    ctx.fillStyle = "#f97316";

    ctx.lineWidth = 3;

    ctx.beginPath();

    ctx.moveTo(craftX, craftY);

    ctx.lineTo(arrowX, arrowY);

    ctx.stroke();


    const arrowAngle = Math.atan2(
      arrowY - craftY,
      arrowX - craftX
    );

    ctx.beginPath();

    ctx.moveTo(arrowX, arrowY);

    ctx.lineTo(
      arrowX - 9 * Math.cos(arrowAngle - .5),
      arrowY - 9 * Math.sin(arrowAngle - .5)
    );

    ctx.lineTo(
      arrowX - 9 * Math.cos(arrowAngle + .5),
      arrowY - 9 * Math.sin(arrowAngle + .5)
    );

    ctx.closePath();

    ctx.fill();

  }


  // ПОСТРОЕНИЕ ГРАФИКА ВЫСОТЫ

  function drawChart(index) {

    const {
      context: ctx,
      width,
      height
    } = fitCanvas(chartCanvas);

    ctx.clearRect(0, 0, width, height);

    const margin = {
      left: width < 380 ? 48 : 62,
      right: width < 380 ? 10 : 18,
      top: 18,
      bottom: 42
    };

    const plotW =
      width - margin.left - margin.right;

    const plotH =
      height - margin.top - margin.bottom;

    const records = result ? result.records : [];

    const visible = records.slice(0, index + 1);

    const target = result
      ? result.config.targetAltitude
      : Number($("targetAltitude").value);

    const initial = result
      ? result.config.initialAltitude
      : Number($("initialAltitude").value);

    const maxTime = Math.max(
      result ? result.final.t / 86400 : 1,
      1e-6
    );

    const values = visible.map(
      (point) => point.altitudeKm
    );

    const minY =
      Math.min(initial, ...values, target) - 30;

    const maxY =
      Math.max(initial, ...values, target) + 30;


    ctx.font = "12px Segoe UI, sans-serif";

    ctx.fillStyle = "#526077";

    ctx.strokeStyle = "#e1e5f6";

    ctx.lineWidth = 1;


    // ГОРИЗОНТАЛЬНЫЕ ЛИНИИ

    for (let tick = 0; tick <= 4; tick += 1) {

      const y = margin.top + plotH * tick / 4;

      const value =
        maxY - (maxY - minY) * tick / 4;

      ctx.beginPath();

      ctx.moveTo(margin.left, y);

      ctx.lineTo(width - margin.right, y);

      ctx.stroke();

      ctx.fillText(
        `${Math.round(value)} км`,
        6,
        y + 4
      );

    }


    // ВРЕМЕННАЯ ШКАЛА

    for (let tick = 0; tick <= 4; tick += 1) {

      const x = margin.left + plotW * tick / 4;

      ctx.fillText(
        `${numberFormat.format(maxTime * tick / 4)} сут`,
        x - 14,
        height - 14
      );

    }


    const yFor = (value) =>
      margin.top +
      (maxY - value) / (maxY - minY) * plotH;


    // ЦЕЛЕВАЯ ВЫСОТА

    ctx.setLineDash([7, 6]);

    ctx.strokeStyle = "#ea580c";

    ctx.beginPath();

    ctx.moveTo(
      margin.left,
      yFor(target)
    );

    ctx.lineTo(
      width - margin.right,
      yFor(target)
    );

    ctx.stroke();

    ctx.setLineDash([]);


    // ФАКТИЧЕСКАЯ ТРАЕКТОРИЯ

    if (visible.length > 1) {

      ctx.beginPath();

      visible.forEach((point, pointIndex) => {

        const x =
          margin.left +
          (point.t / 86400) / maxTime * plotW;

        const y = yFor(point.altitudeKm);

        if (pointIndex === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }

      });

      ctx.strokeStyle = "#4f46e5";

      ctx.lineWidth = 3;

      ctx.stroke();

    }


    ctx.fillStyle = "#4f46e5";

    ctx.fillText(
      "Фактическая высота",
      margin.left + 8,
      margin.top + 14
    );

    ctx.fillStyle = "#c2410c";

    ctx.fillText(
      "Цель",
      width - margin.right - 42,
      yFor(target) - 7
    );

  }


  // ФОРМАТИРОВАНИЕ ВРЕМЕНИ

  function formatTime(seconds) {

    const days = Math.floor(seconds / 86400);

    const hours = Math.floor(
      (seconds % 86400) / 3600
    );

    const minutes = Math.floor(
      (seconds % 3600) / 60
    );

    return days > 0
      ? `${days} сут ${hours} ч`
      : `${hours} ч ${String(minutes).padStart(2, "0")} мин`;

  }


  // ОБНОВЛЕНИЕ ПОКАЗАТЕЛЕЙ

  function updateMetrics(record, index) {

    const config = result
      ? result.config
      : configFromForm();

    $("timeMetric").textContent =
      formatTime(record.t);

    $("progressMetric").textContent =
      result
        ? `${Math.round(index / Math.max(1, result.records.length - 1) * 100)}% сценария`
        : "0% сценария";

    $("altitudeMetric").textContent =
      `${numberFormat.format(record.altitudeKm)} км`;

    $("targetMetric").textContent =
      `цель ${numberFormat.format(config.targetAltitude)} км`;

    $("speedMetric").textContent =
      `${numberFormat.format(record.speedKms)} км/с`;

    $("fuelMetric").textContent =
      `${numberFormat.format(record.fuel)} кг`;

    $("consumedMetric").textContent =
      `израсходовано ${numberFormat.format(config.propellant - record.fuel)} кг`;

  }


  // ОТОБРАЖЕНИЕ ТЕКУЩЕГО КАДРА

  function renderFrame(index) {

    if (!result) {

      drawOrbit(0);
      drawChart(0);

      return;

    }

    const safeIndex = Math.max(
      0,
      Math.min(index, result.records.length - 1)
    );

    updateMetrics(
      result.records[safeIndex],
      safeIndex
    );

    drawOrbit(safeIndex);

    drawChart(safeIndex);

  }


  // ОТОБРАЖЕНИЕ ИТОГОВ МИССИИ

  function renderOutcome() {

    const final = result.final;

    const badge = $("resultBadge");

    badge.className =
      `result-badge ${result.success ? "success" : "failure"}`;

    badge.textContent = result.success
      ? "Миссия успешна"
      : "Критерии не выполнены";


    const reasonText = {

      target: "автостоп у целевой орбиты",

      time: "достигнут лимит времени",

      reentry: "опасное снижение",

      escape: "аппарат покинул связанную орбиту"

    }[result.reason];


    $("resultSummary").textContent =
      `Завершение: ${reasonText}. Средняя высота ${numberFormat.format(final.meanAltitudeKm)} км; эксцентриситет ${final.eccentricity.toFixed(4)}; перицентр ${numberFormat.format(final.periapsisKm)} км; расход ${numberFormat.format(result.consumed)} кг за ${formatTime(final.t)}.`;


    const criteria = [

      [
        result.criteria.target,
        "Высота цели",
        `Ошибка ${numberFormat.format(Math.abs(final.meanAltitudeKm - result.config.targetAltitude))} км при допуске ±${numberFormat.format(result.toleranceKm)} км.`
      ],

      [
        result.criteria.circular,
        "Форма орбиты",
        `Эксцентриситет ${final.eccentricity.toFixed(4)}; требуется ≤ 0,05.`
      ],

      [
        result.criteria.safe,
        "Безопасный перицентр",
        `${numberFormat.format(final.periapsisKm)} км; требуется ≥ 120 км.`
      ],

      [
        result.criteria.fuel,
        "Запас рабочего тела",
        `Остаток ${numberFormat.format(final.fuel)} кг; израсходовано ${numberFormat.format(result.consumed)} кг.`
      ]

    ];


    $("criteriaList").innerHTML = criteria.map(
      ([pass, title, detail]) => `
        <div class="criterion ${pass ? "pass" : "fail"}">
          <span aria-hidden="true">${pass ? "✓" : "×"}</span>
          <div>
            <strong>${title}: ${pass ? "выполнено" : "не выполнено"}</strong>
            <small>${detail}</small>
          </div>
        </div>
      `
    ).join("");


    $("trendText").textContent =
      `Изменение высоты: ${numberFormat.format(final.altitudeKm - result.config.initialAltitude)} км. Средняя орбитальная высота: ${numberFormat.format(final.meanAltitudeKm)} км.`;

  }


  // УПРАВЛЕНИЕ СОСТОЯНИЕМ МОДЕЛИРОВАНИЯ

  function setRunningState(isRunning, statusText) {

    $("pauseButton").disabled = !isRunning;

    $("startButton").textContent = isRunning
      ? "Перезапустить расчёт"
      : "Запустить моделирование";

    const status = $("runStatus");

    status.textContent = statusText;

    status.className =
      `status-pill ${isRunning ? "running" : ""}`;

  }


  // АНИМАЦИЯ

  function animate(timestamp) {

    if (!root.isConnected || !result || paused) return;

    if (lastTimestamp === null) {
      lastTimestamp = timestamp;
    }

    const speed = Number($("playbackSpeed").value);

    framePosition +=
      (timestamp - lastTimestamp) *
      speed *
      (result.records.length - 1) / 14000;

    lastTimestamp = timestamp;

    const index = Math.min(
      result.records.length - 1,
      Math.floor(framePosition)
    );

    renderFrame(index);


    if (index >= result.records.length - 1) {

      setRunningState(
        false,
        "Моделирование завершено"
      );

      renderOutcome();

      animationId = null;

      return;

    }

    animationId = global.requestAnimationFrame(animate);

  }


  // ЗАПУСК МОДЕЛИРОВАНИЯ

  function startSimulation(event) {

    event.preventDefault();

    const config = configFromForm();

    const emptyField = Array.from(
      form.querySelectorAll('input[type="number"]')
    ).find(
      input => input.value.trim() === ""
    );

    const error = emptyField
      ? "Заполните все числовые поля."
      : validate(config);


    $("formError").hidden = !error;

    $("formError").textContent = error;

    if (error) return;


    if (animationId) {
      global.cancelAnimationFrame(animationId);
    }

    result = simulateMission(config);

    framePosition = 0;

    lastTimestamp = null;

    paused = false;

    $("pauseButton").textContent = "Пауза";


    setRunningState(
      true,
      "Тяга включена · расчёт готов"
    );


    if (prefersReducedMotion) {

      framePosition = result.records.length - 1;

      renderFrame(framePosition);

      setRunningState(
        false,
        "Моделирование завершено"
      );

      renderOutcome();

    } else {

      animationId =
        global.requestAnimationFrame(animate);

    }

  }


  // ПАУЗА И ПРОДОЛЖЕНИЕ

  function togglePause() {

    if (
      !result ||
      framePosition >= result.records.length - 1
    ) {
      return;
    }

    paused = !paused;

    $("pauseButton").textContent =
      paused ? "Продолжить" : "Пауза";

    $("runStatus").textContent =
      paused
        ? "Просмотр приостановлен"
        : "Тяга включена · просмотр";

    $("runStatus").className =
      `status-pill ${paused ? "paused" : "running"}`;


    if (!paused) {

      lastTimestamp = null;

      animationId =
        global.requestAnimationFrame(animate);

    }

  }


  // СБРОС ПАРАМЕТРОВ

  function resetDefaults() {

    form.reset();

    $("angleOutput").textContent = "0°";

    if (animationId) {
      global.cancelAnimationFrame(animationId);
    }

    result = null;

    framePosition = 0;

    paused = false;


    $("resultBadge").className =
      "result-badge neutral";

    $("resultBadge").textContent = "Нет расчёта";

    $("resultSummary").textContent =
      "Численный итог появится после завершения моделирования.";


    $("criteriaList")
      .querySelectorAll(".criterion")
      .forEach((item) => {

        item.className = "criterion pending";

        item.querySelector("span").textContent = "—";

      });


    setRunningState(
      false,
      "Готово к запуску"
    );


    const config = configFromForm();


    updateMetrics({

      t: 0,

      altitudeKm: config.initialAltitude,

      speedKms:
        Math.sqrt(
          MU /
          (EARTH_RADIUS + config.initialAltitude * 1000)
        ) / 1000,

      fuel: config.propellant

    }, 0);


    drawOrbit(0);

    drawChart(0);

  }


  // ПОДКЛЮЧЕНИЕ ОБРАБОТЧИКОВ СОБЫТИЙ

  form.addEventListener(
    "submit",
    startSimulation
  );

  $("pauseButton").addEventListener(
    "click",
    togglePause
  );

  $("defaultButton").addEventListener(
    "click",
    resetDefaults
  );


  $("thrustAngle").addEventListener(
    "input",
    (event) => {

      $("angleOutput").textContent =
        `${event.target.value}°`;

      if (!result) {
        drawOrbit(0);
      }

    }
  );


  ["initialAltitude", "targetAltitude", "propellant"]
    .forEach((id) => {

      $(id).addEventListener(
        "input",
        () => {

          if (!result) {
            resetDefaultsPreview();
          }

        }
      );

    });


  // ПРЕДВАРИТЕЛЬНОЕ ОБНОВЛЕНИЕ ПОКАЗАТЕЛЕЙ

  function resetDefaultsPreview() {

    const config = configFromForm();

    if (
      !Number.isFinite(config.initialAltitude) ||
      config.initialAltitude < 0 ||
      !Number.isFinite(config.targetAltitude) ||
      config.targetAltitude <= 0 ||
      !Number.isFinite(config.propellant) ||
      config.propellant < 0
    ) {
      return;
    }


    updateMetrics({

      t: 0,

      altitudeKm: config.initialAltitude,

      speedKms:
        Math.sqrt(
          MU /
          (EARTH_RADIUS + config.initialAltitude * 1000)
        ) / 1000,

      fuel: config.propellant

    }, 0);


    drawOrbit(0);

    drawChart(0);

  }


  // ПОСЛЕДОВАТЕЛЬНЫЙ ПЕРЕХОД МЕЖДУ ПОЛЯМИ
  // СЛЕВА НАПРАВО, СВЕРХУ ВНИЗ

  const orderedFields = Array.from(
    form.querySelectorAll(
      'input[type="number"], select'
    )
  );

  form.addEventListener(
    "keydown",
    event => {

      if (
        event.key !== "Enter" ||
        event.shiftKey ||
        event.isComposing
      ) {
        return;
      }

      const index = orderedFields.indexOf(
        event.target
      );

      if (index < 0) return;

      event.preventDefault();


      if (index + 1 < orderedFields.length) {

        orderedFields[index + 1].focus();

      } else {

        event.target.blur();

        $("startButton").focus();

      }

    }
  );


  // АДАПТАЦИЯ ПРИ ИЗМЕНЕНИИ РАЗМЕРОВ ЭКРАНА

  if (global.ResizeObserver) {

    new ResizeObserver(
      () => renderFrame(Math.floor(framePosition))
    ).observe(root);

  }


  // ПЕРВИЧНАЯ ИНИЦИАЛИЗАЦИЯ

  resetDefaults();

})(typeof window !== "undefined" ? window : globalThis);
