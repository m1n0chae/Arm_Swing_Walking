%% Data load

load('merged_stride_grf_gyro')
load('merged_stride_kinematics_arm_vector_gyro')
load('merged_stride_kinematics_arm_gyro')


%% Figure3) 속도별 P2P and Offset 

% ===== 기본 설정 =====
subjects = {'S001','S002','S003','S004','S005','S006','S007','S008','S009',...
    'S010','S011','S012','S013','S014','S015','S016','S017','S018','S019','S020'};
sessions = {'ss1','ss2','ss3'};
speed_map = table({'ss1';'ss2';'ss3'}, [1.0; 1.25; 1.5], ...
                  'VariableNames', {'Session','Speed'});
REL_WE = 'L_Wrist_to_L_Elbow';
REL_ES = 'L_Elbow_to_L_Shoulder';
REL_SC = 'L_Shoulder_to_sacrum';

% ============================================================
%  [사용자 설정]
% ============================================================
SHOW_XLABEL   = false;
SHOW_YLABEL   = false;

% --- 스케일 팩터 모드: 'p2p' | 'offset' ---
SCALE_MODE = 'p2p';

scale_p2p.ss1.ES = 0.5341;  scale_p2p.ss1.SC = 0.1286;
scale_p2p.ss2.ES = 0.4721;  scale_p2p.ss2.SC = 0.1024;
scale_p2p.ss3.ES = 0.4340;  scale_p2p.ss3.SC = 0.0849;

scale_offset.ss1.ES = 0.6904;  scale_offset.ss1.SC = -0.8697;
scale_offset.ss2.ES = 0.6812;  scale_offset.ss2.SC = -0.8696;
scale_offset.ss3.ES = 0.6729;  scale_offset.ss3.SC = -0.8713;

% --- 오프셋 설정 ---
OFFSET_MODE   = 'manual';
MANUAL_OFF_WE = deg2rad(122.85);
MANUAL_OFF_ES = deg2rad(83.57);
MANUAL_OFF_SC = deg2rad(-106.80);

fs = 100;

% --- P2P Figure ---
ylim_p2p   = [0 90];
ytick_p2p  = 0:30:90;

% --- Offset Figure ---
ylim_off      = [60 150];
ytick_off     = 60:30:150;
AUTO_YLIM_OFF = false;   % true: 데이터 기반 자동 / false: 수동

% ============================================================
%  Figure 스타일 설정
% ============================================================
FONT_NAME      = 'Helvetica';
FONT_SIZE      = 8;
linewidth_axis = 0.8;
linewidth_line = 1.5;
MARKER_SZ      = 4.5;
CAP_SIZE       = 0;
fig_size       = [300 300 190 170];

col_WE = [230  81   0] / 255;
col_ES = [112  48 160] / 255;
col_SC = [ 46 126  50] / 255;

% ========================================================================
% 1. 데이터 수집 (측정값 + 추정값 동시)
% ========================================================================
rows_p2p        = {};
rows_offset     = {};
rows_p2p_est    = {};
rows_offset_est = {};

for ii = 1:numel(subjects)
    subj = subjects{ii};
    if ~isfield(stride_kinematics_arm_vector.(REL_WE), subj), continue; end

    for jj = 1:numel(sessions)
        sess = sessions{jj};
        if ~isfield(stride_kinematics_arm_vector.(REL_WE).(subj), sess), continue; end

        % 세션별 스케일 팩터 선택
        if strcmpi(SCALE_MODE, 'p2p')
            if isfield(scale_p2p, sess)
                ES_SCALE = scale_p2p.(sess).ES;
                SC_SCALE = scale_p2p.(sess).SC;
            else
                ES_SCALE = scale_p2p.ss2.ES;
                SC_SCALE = scale_p2p.ss2.SC;
            end
        else
            if isfield(scale_offset, sess)
                ES_SCALE = scale_offset.(sess).ES;
                SC_SCALE = scale_offset.(sess).SC;
            else
                ES_SCALE = scale_offset.ss2.ES;
                SC_SCALE = scale_offset.ss2.SC;
            end
        end

        days = fieldnames(stride_kinematics_arm_vector.(REL_WE).(subj).(sess));

        for dd = 1:numel(days)
            day = days{dd};

            has_WE = isfield(stride_kinematics_arm_vector.(REL_WE).(subj).(sess), day) && ...
                     isfield(stride_kinematics_arm_vector.(REL_WE).(subj).(sess).(day), 'sagittal_angle');
            has_ES = isfield(stride_kinematics_arm_vector.(REL_ES).(subj).(sess), day) && ...
                     isfield(stride_kinematics_arm_vector.(REL_ES).(subj).(sess).(day), 'sagittal_angle');
            has_SC = isfield(stride_kinematics_arm_vector.(REL_SC).(subj).(sess), day) && ...
                     isfield(stride_kinematics_arm_vector.(REL_SC).(subj).(sess).(day), 'sagittal_angle');
            if ~(has_WE && has_ES && has_SC), continue; end

            % 추정값용 gyro 접근
            has_gyro = isfield(stride_kinematics_arm, 'L_Wrist') && ...
                       isfield(stride_kinematics_arm.L_Wrist, subj) && ...
                       isfield(stride_kinematics_arm.L_Wrist.(subj), sess) && ...
                       isfield(stride_kinematics_arm.L_Wrist.(subj).(sess), day) && ...
                       isfield(stride_kinematics_arm.L_Wrist.(subj).(sess).(day), 'imu_gyro_local_y');
            gyroLeaf = [];
            if has_gyro
                gyroLeaf = stride_kinematics_arm.L_Wrist.(subj).(sess).(day);
            end

            cell_WE = stride_kinematics_arm_vector.(REL_WE).(subj).(sess).(day).sagittal_angle.pos;
            cell_ES = stride_kinematics_arm_vector.(REL_ES).(subj).(sess).(day).sagittal_angle.pos;
            cell_SC = stride_kinematics_arm_vector.(REL_SC).(subj).(sess).(day).sagittal_angle.pos;

            n_strides = min([numel(cell_WE), numel(cell_ES), numel(cell_SC)]);
            if n_strides == 0, continue; end

            for k = 1:n_strides
                % ── 측정값 P2P / Offset ──────────────────────
                if ~isempty(cell_WE{k})
                    d = cell_WE{k} * (180/pi);
                    p2p_WE = max(d) - min(d); off_WE = mean(d,'omitnan');
                else
                    p2p_WE = NaN; off_WE = NaN;
                end
                if ~isempty(cell_ES{k})
                    d = cell_ES{k} * (180/pi);
                    p2p_ES = max(d) - min(d); off_ES = mean(d,'omitnan');
                else
                    p2p_ES = NaN; off_ES = NaN;
                end
                if ~isempty(cell_SC{k})
                    d = cell_SC{k} * (180/pi);
                    p2p_SC = max(d) - min(d); off_SC = mean(d,'omitnan');
                else
                    p2p_SC = NaN; off_SC = NaN;
                end
                rows_p2p(end+1,:)    = {subj, sess, day, k, p2p_WE, p2p_ES, p2p_SC};  %#ok<SAGROW>
                rows_offset(end+1,:) = {subj, sess, day, k, off_WE, off_ES, off_SC};   %#ok<SAGROW>

                % ── 추정값 P2P / Offset ──────────────────────
                p2p_WE_e = NaN; p2p_ES_e = NaN; p2p_SC_e = NaN;
                off_WE_e = NaN; off_ES_e = NaN; off_SC_e = NaN;

                if has_gyro && iscell(gyroLeaf.imu_gyro_local_y) && ...
                   numel(gyroLeaf.imu_gyro_local_y) >= k && ...
                   ~isempty(gyroLeaf.imu_gyro_local_y{k})

                    gy = gyroLeaf.imu_gyro_local_y{k}(:);
                    N  = numel(gy);
                    t  = (0:N-1)' / fs;

                    % 바이어스 제거
                    idxF = isfinite(gy);
                    if any(idxF), gy = gy - median(gy(idxF)); end

                    % 각도 추정
                    th_we = cumtrapz(t, gy);
                    th_we = th_we - th_we(1);
                    th_es = ES_SCALE * th_we;
                    th_sc = SC_SCALE * th_we;

                    % 오프셋 적용
                    switch lower(OFFSET_MODE)
                        case 'manual'
                            th_we_fd = th_we + MANUAL_OFF_WE;
                            th_es_fd = th_es + MANUAL_OFF_ES;
                            th_sc_fd = th_sc + MANUAL_OFF_SC;
                        case 'off'
                            th_we_fd = th_we;
                            th_es_fd = th_es;
                            th_sc_fd = th_sc;
                        otherwise
                            th_we_fd = th_we + MANUAL_OFF_WE;
                            th_es_fd = th_es + MANUAL_OFF_ES;
                            th_sc_fd = th_sc + MANUAL_OFF_SC;
                    end

                    % deg 변환 후 P2P / Offset
                    we_d = th_we_fd * (180/pi);
                    es_d = th_es_fd * (180/pi);
                    sc_d = th_sc_fd * (180/pi);

                    p2p_WE_e = max(we_d) - min(we_d);
                    p2p_ES_e = max(es_d) - min(es_d);
                    p2p_SC_e = max(sc_d) - min(sc_d);
                    off_WE_e = mean(we_d, 'omitnan');
                    off_ES_e = mean(es_d, 'omitnan');
                    off_SC_e = mean(sc_d, 'omitnan');
                end

                rows_p2p_est(end+1,:)    = {subj, sess, day, k, p2p_WE_e, p2p_ES_e, p2p_SC_e};  %#ok<SAGROW>
                rows_offset_est(end+1,:) = {subj, sess, day, k, off_WE_e, off_ES_e, off_SC_e};   %#ok<SAGROW>
            end
        end
    end
end
fprintf('수집된 stride 수: %d\n', size(rows_p2p, 1));

% ========================================================================
% 2. 집계 함수
% ========================================================================
function T_abs = make_abs_table(rows, speed_map)
    T_raw = cell2table(rows, 'VariableNames', ...
        {'Subject','Session','Day','StrideIdx','WE','ES','SC'});
    [G, Sub, Sess, Day] = findgroups(T_raw.Subject, T_raw.Session, T_raw.Day);
    T_td = table(Sub, Sess, Day, ...
        splitapply(@(x) mean(x,'omitnan'), T_raw.WE, G), ...
        splitapply(@(x) mean(x,'omitnan'), T_raw.ES, G), ...
        splitapply(@(x) mean(x,'omitnan'), T_raw.SC, G), ...
        'VariableNames', {'Subject','Session','Day','WE','ES','SC'});
    [G2, Sub2, Sess2] = findgroups(T_td.Subject, T_td.Session);
    T_trial = table(Sub2, Sess2, ...
        splitapply(@(x) mean(x,'omitnan'), T_td.WE, G2), ...
        splitapply(@(x) mean(x,'omitnan'), T_td.ES, G2), ...
        splitapply(@(x) mean(x,'omitnan'), T_td.SC, G2), ...
        'VariableNames', {'Subject','Session','WE','ES','SC'});
    T_trial = innerjoin(T_trial, speed_map, 'Keys','Session');
    [Gsess, SessList] = findgroups(T_trial.Session);
    T_abs = table(SessList, ...
        splitapply(@mean, T_trial.WE, Gsess), ...
        splitapply(@std,  T_trial.WE, Gsess) ./ sqrt(splitapply(@numel, T_trial.WE, Gsess)), ...
        splitapply(@mean, T_trial.ES, Gsess), ...
        splitapply(@std,  T_trial.ES, Gsess) ./ sqrt(splitapply(@numel, T_trial.ES, Gsess)), ...
        splitapply(@mean, abs(T_trial.SC), Gsess), ...
        splitapply(@std,  abs(T_trial.SC), Gsess) ./ sqrt(splitapply(@numel, T_trial.SC, Gsess)), ...
        'VariableNames', {'Session','WE_m','WE_sem','ES_m','ES_sem','SC_m','SC_sem'});
    T_abs = innerjoin(T_abs, speed_map, 'Keys','Session');
    T_abs = sortrows(T_abs, 'Speed');
end

% ========================================================================
% 3. 측정값 / 추정값 통계 테이블 생성
% ========================================================================
T_P2P_abs = make_abs_table(rows_p2p,        speed_map);
T_P2P_est = make_abs_table(rows_p2p_est,    speed_map);
T_Off_abs  = make_abs_table(rows_offset,     speed_map);
T_Off_est  = make_abs_table(rows_offset_est, speed_map);

fprintf('P2P Trial 수 (측정): %d\n', size(rows_p2p, 1));

% ========================================================================
% 4. Figure 생성
% ========================================================================
speeds_unique = unique(T_P2P_abs.Speed);
x_lim = [min(speeds_unique)-0.1, max(speeds_unique)+0.1];

apply_style = @(ax, yt) set(ax, ...
    'FontName',  FONT_NAME, ...
    'FontSize',  FONT_SIZE, ...
    'Box',       'off', ...
    'TickDir',   'out', ...
    'LineWidth', linewidth_axis, ...
    'XLim',      x_lim, ...
    'XTick',     speeds_unique, ...
    'YTick',     yt);

% ── P2P Figure ────────────────────────────────────────────────
figure('Color', 'w', 'Units', 'pixels', 'Position', fig_size, ...
    'Name', 'P2P Absolute');
hold on; box off;

% 측정값 (실선 + 채운 마커)
errorbar(T_P2P_abs.Speed, T_P2P_abs.WE_m, T_P2P_abs.WE_sem, ...
    'LineStyle', '-', 'Color', col_WE, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', col_WE, 'MarkerEdgeColor', col_WE, 'CapSize', CAP_SIZE);
errorbar(T_P2P_abs.Speed, T_P2P_abs.ES_m, T_P2P_abs.ES_sem, ...
    'LineStyle', '-', 'Color', col_ES, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', col_ES, 'MarkerEdgeColor', col_ES, 'CapSize', CAP_SIZE);
errorbar(T_P2P_abs.Speed, T_P2P_abs.SC_m, T_P2P_abs.SC_sem, ...
    'LineStyle', '-', 'Color', col_SC, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', col_SC, 'MarkerEdgeColor', col_SC, 'CapSize', CAP_SIZE);

% 추정값 (점선 + 빈 마커)
errorbar(T_P2P_est.Speed, T_P2P_est.WE_m, T_P2P_est.WE_sem, ...
    'LineStyle', '--', 'Color', col_WE, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', col_WE, 'CapSize', CAP_SIZE);
errorbar(T_P2P_est.Speed, T_P2P_est.ES_m, T_P2P_est.ES_sem, ...
    'LineStyle', '--', 'Color', col_ES, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', col_ES, 'CapSize', CAP_SIZE);
errorbar(T_P2P_est.Speed, T_P2P_est.SC_m, T_P2P_est.SC_sem, ...
    'LineStyle', '--', 'Color', col_SC, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', col_SC, 'CapSize', CAP_SIZE);

apply_style(gca, ytick_p2p);
ylim(ylim_p2p);
if SHOW_XLABEL, xlabel('Speed (m/s)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE); end
if SHOW_YLABEL, ylabel('P2P Amplitude (deg)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE); end
ax = gca; ax.Position = [0.22 0.18 0.74 0.76];
hold off;
disp('=== P2P Figure 완료 ===');

% ── Offset Figure ─────────────────────────────────────────────
figure('Color', 'w', 'Units', 'pixels', 'Position', fig_size, ...
    'Name', 'Offset Absolute');
hold on; box off;

% 측정값 (실선 + 채운 마커)
errorbar(T_Off_abs.Speed, T_Off_abs.WE_m, T_Off_abs.WE_sem, ...
    'LineStyle', '-', 'Color', col_WE, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', col_WE, 'MarkerEdgeColor', col_WE, 'CapSize', CAP_SIZE);
errorbar(T_Off_abs.Speed, T_Off_abs.ES_m, T_Off_abs.ES_sem, ...
    'LineStyle', '-', 'Color', col_ES, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', col_ES, 'MarkerEdgeColor', col_ES, 'CapSize', CAP_SIZE);
errorbar(T_Off_abs.Speed, T_Off_abs.SC_m, T_Off_abs.SC_sem, ...
    'LineStyle', '-', 'Color', col_SC, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', col_SC, 'MarkerEdgeColor', col_SC, 'CapSize', CAP_SIZE);

% 추정값 (점선 + 빈 마커)
errorbar(T_Off_est.Speed, T_Off_est.WE_m, T_Off_est.WE_sem, ...
    'LineStyle', '--', 'Color', col_WE, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', col_WE, 'CapSize', CAP_SIZE);
errorbar(T_Off_est.Speed, T_Off_est.ES_m, T_Off_est.ES_sem, ...
    'LineStyle', '--', 'Color', col_ES, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', col_ES, 'CapSize', CAP_SIZE);
errorbar(T_Off_est.Speed, T_Off_est.SC_m, T_Off_est.SC_sem, ...
    'LineStyle', '--', 'Color', col_SC, 'LineWidth', linewidth_line, ...
    'Marker', 'o', 'MarkerSize', MARKER_SZ, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', col_SC, 'CapSize', CAP_SIZE);

if AUTO_YLIM_OFF
    all_tops = [T_Off_abs.WE_m + T_Off_abs.WE_sem; ...
                T_Off_abs.ES_m + T_Off_abs.ES_sem; ...
                T_Off_abs.SC_m + T_Off_abs.SC_sem; ...
                T_Off_est.WE_m + T_Off_est.WE_sem; ...
                T_Off_est.ES_m + T_Off_est.ES_sem; ...
                T_Off_est.SC_m + T_Off_est.SC_sem];
    y_max    = max(all_tops) * 1.2;
    ylim_off = [0, ceil(y_max/10)*10];
    ytick_off = 0 : (ylim_off(2)/4) : ylim_off(2);
end

apply_style(gca, ytick_off);
ylim(ylim_off);
if SHOW_XLABEL, xlabel('Speed (m/s)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE); end
if SHOW_YLABEL, ylabel('Offset (deg)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE); end
ax = gca; ax.Position = [0.22 0.18 0.74 0.76];
hold off;
disp('=== Offset Figure 완료 ===');


%% Figure 4-1) Segment별 가속도 추정 (계산 파트)
% ================================================================
% A) DATA 준비: 모든 subj/session/day/stride 수집 → 시간정규화 → 집계
%     + [수정됨] 속도(Session)별 ES_SCALE, SC_SCALE 적용
%     + [수정됨] Subject별 통계 (mean ± SD across subjects)
%     + [수정됨] M_sess_label 추가
% ================================================================
clc; % close all;
% ---------------- 사용자 옵션 ----------------
fs  = 100;             % Hz
dt  = 1/fs;
% ============================================================
% [수정] 스케일 팩터 모드: 'p2p' | 'offset'
% ============================================================
SCALE_MODE = 'p2p';  % 'p2p' 또는 'offset' 선택
% --- P2P 기반 스케일 팩터 (속도별) ---
scale_p2p.ss1.ES = 0.5341;   scale_p2p.ss1.SC = 0.1286;   % 1.0 m/s
scale_p2p.ss2.ES = 0.4721;   scale_p2p.ss2.SC = 0.1024;   % 1.25 m/s
scale_p2p.ss3.ES = 0.4340;   scale_p2p.ss3.SC = 0.0849;   % 1.5 m/s
% --- Offset 기반 스케일 팩터 (속도별) ---
scale_offset.ss1.ES = 0.6904;   scale_offset.ss1.SC = -0.8697;   % 1.0 m/s
scale_offset.ss2.ES = 0.6812;   scale_offset.ss2.SC = -0.8696;   % 1.25 m/s
scale_offset.ss3.ES = 0.6729;   scale_offset.ss3.SC = -0.8713;   % 1.5 m/s
% IMU 가속도 프레임: 'global' | 'local'
ACC_FRAME = 'global';
% 오프셋 옵션
OFFSET_MODE   = 'manual';           % 'auto' | 'off' | 'manual' | 'gravity'
MANUAL_OFF_WE = deg2rad(122.85);   % mean of [122.47, 122.66, 123.42]
MANUAL_OFF_ES = deg2rad(83.57);    % mean of [84.39, 83.45, 82.88]
MANUAL_OFF_SC = deg2rad(-106.80);   % mean of [106.25, 106.86, 107.30]
% ES/SC 오프셋 스케일(we 오프셋의 배수)
es_offset_scaling_factor = 0.75;
sc_offset_scaling_factor = -0.77;
APPLY_OFFSET_TO_FORWARD = true;
% 처리 대상 필터
target_subjects = {};
target_sessions = {};
target_days     = {};
stride_range    = [];
% 시간정규화 샘플
Nnorm  = 101;
phase_tgt = linspace(0,1,Nnorm);
ph_pct    = linspace(0,100,Nnorm);
% ---------------- 사전 체크 ----------------
if ~(exist('stride_kinematics_arm','var')==1 && exist('stride_kinematics_arm_vector','var')==1)
    error('stride_kinematics_arm / stride_kinematics_arm_vector 가 필요합니다.');
end
if ~isfield(stride_kinematics_arm,'L_Wrist')
    error('stride_kinematics_arm.L_Wrist 가 없습니다.');
end
% ---------------- 스케일 모드 출력 ----------------
fprintf('\n=== Scale Factor 설정 (SCALE_MODE: %s) ===\n', SCALE_MODE);
if strcmpi(SCALE_MODE, 'p2p')
    fprintf('  ss1 (1.0 m/s):  ES=%.4f, SC=%.4f\n', scale_p2p.ss1.ES, scale_p2p.ss1.SC);
    fprintf('  ss2 (1.25 m/s): ES=%.4f, SC=%.4f\n', scale_p2p.ss2.ES, scale_p2p.ss2.SC);
    fprintf('  ss3 (1.5 m/s):  ES=%.4f, SC=%.4f\n', scale_p2p.ss3.ES, scale_p2p.ss3.SC);
else
    fprintf('  ss1 (1.0 m/s):  ES=%.4f, SC=%.4f\n', scale_offset.ss1.ES, scale_offset.ss1.SC);
    fprintf('  ss2 (1.25 m/s): ES=%.4f, SC=%.4f\n', scale_offset.ss2.ES, scale_offset.ss2.SC);
    fprintf('  ss3 (1.5 m/s):  ES=%.4f, SC=%.4f\n', scale_offset.ss3.ES, scale_offset.ss3.SC);
end
fprintf('\n');
% ---------------- 집계 버퍼 ----------------
M_wrist_x = []; M_wrist_z = [];
M_inc_we_x = []; M_inc_we_z = [];
M_inc_es_x = []; M_inc_es_z = [];
M_inc_sc_x = []; M_inc_sc_z = [];
M_cum_x    = []; M_cum_z    = [];
M_sac_x    = []; M_sac_z    = [];
M_wrist_true_x = []; M_wrist_true_z = [];
M_elbow_true_x = []; M_elbow_true_z = [];
M_shldr_true_x = []; M_shldr_true_z = [];
M_inc_we_true_x = []; M_inc_we_true_z = [];
M_inc_es_true_x = []; M_inc_es_true_z = [];
M_inc_sc_true_x = []; M_inc_sc_true_z = [];
M_sum_comp_x = []; M_sum_comp_z = [];
% [추가] Subject / Session 라벨 저장 벡터
M_subj_label = {};   % 각 stride 행에 대응하는 subject 이름
M_sess_label = {};   % 각 stride 행에 대응하는 session 이름  ★ 추가
n_stride_total = 0; n_stride_used = 0; n_stride_skipped = 0;
n_stride_with_sacrum = 0;
% ---------------- 루프: Subject / Session / Day ----------------
all_subj = fieldnames(stride_kinematics_arm.L_Wrist).';
if ~isempty(target_subjects), all_subj = all_subj(ismember(all_subj, target_subjects)); end
for iS = 1:numel(all_subj)
    subj = all_subj{iS};
    if ~isfield(stride_kinematics_arm.L_Wrist, subj), continue; end
    
    all_sess = fieldnames(stride_kinematics_arm.L_Wrist.(subj)).';
    if ~isempty(target_sessions), all_sess = all_sess(ismember(all_sess, target_sessions)); end
    
    for iSe = 1:numel(all_sess)
        sess = all_sess{iSe};
        if ~isfield(stride_kinematics_arm.L_Wrist.(subj), sess), continue; end
        
        % ============================================================
        % [수정] Session별 스케일 팩터 선택
        % ============================================================
        if strcmpi(SCALE_MODE, 'p2p')
            if isfield(scale_p2p, sess)
                ES_SCALE = scale_p2p.(sess).ES;
                SC_SCALE = scale_p2p.(sess).SC;
            else
                warning('scale_p2p에 %s 없음. ss2 사용', sess);
                ES_SCALE = scale_p2p.ss2.ES;
                SC_SCALE = scale_p2p.ss2.SC;
            end
        else  % 'offset'
            if isfield(scale_offset, sess)
                ES_SCALE = scale_offset.(sess).ES;
                SC_SCALE = scale_offset.(sess).SC;
            else
                warning('scale_offset에 %s 없음. ss2 사용', sess);
                ES_SCALE = scale_offset.ss2.ES;
                SC_SCALE = scale_offset.ss2.SC;
            end
        end
        
        all_days = fieldnames(stride_kinematics_arm.L_Wrist.(subj).(sess)).';
        isDay = ~cellfun('isempty', regexp(all_days,'^Day','once'));
        all_days = all_days(isDay);
        if ~isempty(target_days), all_days = all_days(ismember(all_days, target_days)); end
        
        for iD = 1:numel(all_days)
            day = all_days{iD};
            
            % leaf
            if ~isfield(stride_kinematics_arm.L_Wrist.(subj).(sess), day), continue; end
            if ~isfield(stride_kinematics_arm.L_Elbow.(subj).(sess), day), continue; end
            if ~isfield(stride_kinematics_arm.L_Shoulder.(subj).(sess), day), continue; end
            wLeaf = stride_kinematics_arm.L_Wrist.(subj).(sess).(day);
            eLeaf = stride_kinematics_arm.L_Elbow.(subj).(sess).(day);
            sLeaf = stride_kinematics_arm.L_Shoulder.(subj).(sess).(day);
            
            cLeaf = [];
            if isfield(stride_kinematics_arm,'sacrum') && ...
               isfield(stride_kinematics_arm.sacrum, subj) && ...
               isfield(stride_kinematics_arm.sacrum.(subj), sess) && ...
               isfield(stride_kinematics_arm.sacrum.(subj).(sess), day)
                cLeaf = stride_kinematics_arm.sacrum.(subj).(sess).(day);
            end
            
            % vector
            haveVec = true;
            if isfield(stride_kinematics_arm_vector,'L_Wrist_to_L_Elbow') && ...
               isfield(stride_kinematics_arm_vector.L_Wrist_to_L_Elbow, subj) && ...
               isfield(stride_kinematics_arm_vector.L_Wrist_to_L_Elbow.(subj), sess) && ...
               isfield(stride_kinematics_arm_vector.L_Wrist_to_L_Elbow.(subj).(sess), day)
                vWE = stride_kinematics_arm_vector.L_Wrist_to_L_Elbow.(subj).(sess).(day);
            else
                haveVec = false;
            end
            if isfield(stride_kinematics_arm_vector,'L_Elbow_to_L_Shoulder') && ...
               isfield(stride_kinematics_arm_vector.L_Elbow_to_L_Shoulder, subj) && ...
               isfield(stride_kinematics_arm_vector.L_Elbow_to_L_Shoulder.(subj), sess) && ...
               isfield(stride_kinematics_arm_vector.L_Elbow_to_L_Shoulder.(subj).(sess), day)
                vES = stride_kinematics_arm_vector.L_Elbow_to_L_Shoulder.(subj).(sess).(day);
            else
                haveVec = false;
            end
            if isfield(stride_kinematics_arm_vector,'L_Shoulder_to_sacrum') && ...
               isfield(stride_kinematics_arm_vector.L_Shoulder_to_sacrum, subj) && ...
               isfield(stride_kinematics_arm_vector.L_Shoulder_to_sacrum.(subj), sess) && ...
               isfield(stride_kinematics_arm_vector.L_Shoulder_to_sacrum.(subj).(sess), day)
                vSC = stride_kinematics_arm_vector.L_Shoulder_to_sacrum.(subj).(sess).(day);
            else
                haveVec = false;
            end
            if ~haveVec, continue; end
            
            % stride 개수
            if ~isfield(wLeaf,'imu_gyro_local_y') || ~iscell(wLeaf.imu_gyro_local_y) || isempty(wLeaf.imu_gyro_local_y)
                continue;
            end
            nStride = numel(wLeaf.imu_gyro_local_y);
            
            % stride 범위
            if isempty(stride_range)
                s1=1; s2=nStride;
            else
                s1 = max(1, stride_range(1)); s2 = min(nStride, stride_range(2));
                if s1 > s2, continue; end
            end
            
            % ================= STRIDE LOOP =================
            for s = s1:s2
                n_stride_total = n_stride_total + 1;
                
                % 필수 체크
                okW = isfield(wLeaf,'pos_x') && iscell(wLeaf.pos_x) && numel(wLeaf.pos_x)>=s && ~isempty(wLeaf.pos_x{s}) && ...
                      isfield(wLeaf,'pos_z') && iscell(wLeaf.pos_z) && numel(wLeaf.pos_z)>=s && ~isempty(wLeaf.pos_z{s});
                okE = isfield(eLeaf,'pos_x') && iscell(eLeaf.pos_x) && numel(eLeaf.pos_x)>=s && ~isempty(eLeaf.pos_x{s}) && ...
                      isfield(eLeaf,'pos_z') && iscell(eLeaf.pos_z) && numel(eLeaf.pos_z)>=s && ~isempty(eLeaf.pos_z{s});
                okS = isfield(sLeaf,'pos_x') && iscell(sLeaf.pos_x) && numel(sLeaf.pos_x)>=s && ~isempty(sLeaf.pos_x{s}) && ...
                      isfield(sLeaf,'pos_z') && iscell(sLeaf.pos_z) && numel(sLeaf.pos_z)>=s && ~isempty(sLeaf.pos_z{s});
                okG = isfield(wLeaf,'imu_gyro_local_y') && iscell(wLeaf.imu_gyro_local_y) && numel(wLeaf.imu_gyro_local_y)>=s && ~isempty(wLeaf.imu_gyro_local_y{s});
                if ~(okW && okE && okS && okG)
                    n_stride_skipped = n_stride_skipped + 1; continue;
                end
                
                wX = wLeaf.pos_x{s}(:);  wZ = wLeaf.pos_z{s}(:);
                eX = eLeaf.pos_x{s}(:);  eZ = eLeaf.pos_z{s}(:);
                sX = sLeaf.pos_x{s}(:);  sZ = sLeaf.pos_z{s}(:);
                gy = wLeaf.imu_gyro_local_y{s}(:);
                
                % Wrist IMU acc
                if strcmpi(ACC_FRAME,'global')
                    haveAX = isfield(wLeaf,'imu_acc_global_x') && iscell(wLeaf.imu_acc_global_x) && numel(wLeaf.imu_acc_global_x)>=s && ~isempty(wLeaf.imu_acc_global_x{s});
                    haveAZ = isfield(wLeaf,'imu_acc_global_z') && iscell(wLeaf.imu_acc_global_z) && numel(wLeaf.imu_acc_global_z)>=s && ~isempty(wLeaf.imu_acc_global_z{s});
                    if ~(haveAX && haveAZ), n_stride_skipped = n_stride_skipped + 1; continue; end
                    ax_w = wLeaf.imu_acc_global_x{s}(:);
                    az_w = wLeaf.imu_acc_global_z{s}(:);
                else
                    haveAX = isfield(wLeaf,'imu_acc_local_x') && iscell(wLeaf.imu_acc_local_x) && numel(wLeaf.imu_acc_local_x)>=s && ~isempty(wLeaf.imu_acc_local_x{s});
                    haveAZ = isfield(wLeaf,'imu_acc_local_z') && iscell(wLeaf.imu_acc_local_z) && numel(wLeaf.imu_acc_local_z)>=s && ~isempty(wLeaf.imu_acc_local_z{s});
                    if ~(haveAX && haveAZ), n_stride_skipped = n_stride_skipped + 1; continue; end
                    ax_w = wLeaf.imu_acc_local_x{s}(:);
                    az_w = wLeaf.imu_acc_local_z{s}(:);
                end
                
                % sacrum IMU
                ax_sac = []; az_sac = [];
                if ~isempty(cLeaf)
                    if strcmpi(ACC_FRAME,'global')
                        if isfield(cLeaf,'imu_acc_global_x') && iscell(cLeaf.imu_acc_global_x) && numel(cLeaf.imu_acc_global_x)>=s && ~isempty(cLeaf.imu_acc_global_x{s})
                            ax_sac = cLeaf.imu_acc_global_x{s}(:);
                        end
                        if isfield(cLeaf,'imu_acc_global_z') && iscell(cLeaf.imu_acc_global_z) && numel(cLeaf.imu_acc_global_z)>=s && ~isempty(cLeaf.imu_acc_global_z{s})
                            az_sac = cLeaf.imu_acc_global_z{s}(:);
                        end
                    else
                        if isfield(cLeaf,'imu_acc_local_x') && iscell(cLeaf.imu_acc_local_x) && numel(cLeaf.imu_acc_local_x)>=s && ~isempty(cLeaf.imu_acc_local_x{s})
                            ax_sac = cLeaf.imu_acc_local_x{s}(:);
                        end
                        if isfield(cLeaf,'imu_acc_local_z') && iscell(cLeaf.imu_acc_local_z) && numel(cLeaf.imu_acc_local_z)>=s && ~isempty(cLeaf.imu_acc_local_z{s})
                            az_sac = cLeaf.imu_acc_local_z{s}(:);
                        end
                    end
                end
                
                % 길이
                okLwe = isfield(vWE,'length_xz') && isfield(vWE.length_xz,'pos') && iscell(vWE.length_xz.pos) && numel(vWE.length_xz.pos)>=s && ~isempty(vWE.length_xz.pos{s});
                okLes = isfield(vES,'length_xz') && isfield(vES.length_xz,'pos') && iscell(vES.length_xz.pos) && numel(vES.length_xz.pos)>=s && ~isempty(vES.length_xz.pos{s});
                okLsc = isfield(vSC,'length_xz') && isfield(vSC.length_xz,'pos') && iscell(vSC.length_xz.pos) && numel(vSC.length_xz.pos)>=s && ~isempty(vSC.length_xz.pos{s});
                if ~(okLwe && okLes && okLsc), n_stride_skipped = n_stride_skipped + 1; continue; end
                L_we = mean(vWE.length_xz.pos{s}(:),'omitnan');
                L_es = mean(vES.length_xz.pos{s}(:),'omitnan');
                L_sc = mean(vSC.length_xz.pos{s}(:),'omitnan');
                
                % 공통 길이 N
                Ls = [numel(wX), numel(wZ), numel(eX), numel(eZ), numel(sX), numel(sZ), numel(gy), numel(ax_w), numel(az_w)];
                if ~isempty(ax_sac), Ls(end+1)=numel(ax_sac); end
                if ~isempty(az_sac), Ls(end+1)=numel(az_sac); end
                N = min(Ls);
                if N < 5, n_stride_skipped = n_stride_skipped + 1; continue; end
                
                % crop
                wX=wX(1:N); wZ=wZ(1:N); eX=eX(1:N); eZ=eZ(1:N); sX=sX(1:N); sZ=sZ(1:N);
                gy=gy(1:N); ax_w=ax_w(1:N); az_w=az_w(1:N);
                if ~isempty(ax_sac), ax_sac = ax_sac(1:N); end
                if ~isempty(az_sac), az_sac = az_sac(1:N); end
                t = (0:N-1)'/fs;
                
                % gyro 바이어스 제거
                idxF = isfinite(gy);
                if any(idxF), gy = gy - median(gy(idxF)); end
                if any(~idxF)
                    if nnz(idxF)>=2
                        gy(~idxF) = interp1(t(idxF), gy(idxF), t(~idxF), 'linear', 'extrap');
                    else
                        gy(~idxF) = 0;
                    end
                end
                
                % Pred 각도 (Session별 스케일 적용)
                th_we_pred = cumtrapz(t, gy); th_we_pred = th_we_pred - th_we_pred(1);
                th_es_pred = ES_SCALE * th_we_pred;
                th_sc_pred = SC_SCALE * th_we_pred;
                
                % True 각
                th_we_true = atan2(eZ - wZ, eX - wX);
                th_es_true = atan2(sZ - eZ, sX - eX);
                
                % ---------------- 오프셋 결정 ----------------
                switch lower(OFFSET_MODE)
                    case 'off'
                        off_we = 0.0; off_es = 0.0; off_sc = 0.0;
                    case 'manual'
                        off_we = MANUAL_OFF_WE;
                        off_es = MANUAL_OFF_ES;
                        off_sc = MANUAL_OFF_SC;
                    case 'auto'
                        d_we = th_we_true - th_we_pred;
                        off_we = atan2(nanmean(sin(d_we)), nanmean(cos(d_we)));
                        off_es = es_offset_scaling_factor * off_we;
                        off_sc = sc_offset_scaling_factor * off_we;
                    case 'gravity'
                        k0 = NaN; target_rad = NaN;
                        if isfield(vWE,'offset_index')
                            if iscell(vWE.offset_index) && numel(vWE.offset_index)>=s && ~isempty(vWE.offset_index{s})
                                k0 = vWE.offset_index{s}(1);
                            elseif isnumeric(vWE.offset_index) && numel(vWE.offset_index)>=s
                                k0 = vWE.offset_index(s);
                            end
                        end
                        if isfield(vWE,'offset_rad')
                            if iscell(vWE.offset_rad) && numel(vWE.offset_rad)>=s && ~isempty(vWE.offset_rad{s})
                                target_rad = vWE.offset_rad{s}(1);
                            elseif isnumeric(vWE.offset_rad) && numel(vWE.offset_rad)>=s
                                target_rad = vWE.offset_rad(s);
                            end
                        end
                        if ~isnan(k0) && ~isnan(target_rad)
                            k0 = max(1, min(N, round(k0)));
                            off_we = atan2(sin(target_rad - th_we_pred(k0)), cos(target_rad - th_we_pred(k0)));
                        else
                            d_we = th_we_true - th_we_pred;
                            off_we = atan2(nanmean(sin(d_we)), nanmean(cos(d_we)));
                        end
                        off_es = es_offset_scaling_factor * off_we;
                        off_sc = sc_offset_scaling_factor * off_we;
                    otherwise
                        d_we = th_we_true - th_we_pred;
                        off_we = atan2(nanmean(sin(d_we)), nanmean(cos(d_we)));
                        off_es = es_offset_scaling_factor * off_we;
                        off_sc = sc_offset_scaling_factor * off_we;
                end
                
                % 전파식 각도
                if APPLY_OFFSET_TO_FORWARD
                    th_we_fd = th_we_pred + off_we;
                    th_es_fd = th_es_pred + off_es;
                    th_sc_fd = th_sc_pred + off_sc;
                else
                    th_we_fd = th_we_pred;
                    th_es_fd = th_es_pred;
                    th_sc_fd = th_sc_pred;
                end
                
                % 각속/각가속도 (Session별 스케일 적용)
                om_we = gy;                 al_we = gradient(om_we, dt);
                om_es = ES_SCALE * om_we;   al_es = ES_SCALE * al_we;
                om_sc = SC_SCALE * om_we;   al_sc = SC_SCALE * al_we;
                
                % 상대가속도 항
                rddx_we = -L_we .* ( cos(th_we_fd).*(om_we.^2) + sin(th_we_fd).*al_we );
                rddz_we = -L_we .* ( sin(th_we_fd).*(om_we.^2) - cos(th_we_fd).*al_we );
                rddx_es = -L_es .* ( cos(th_es_fd).*(om_es.^2) + sin(th_es_fd).*al_es );
                rddz_es = -L_es .* ( sin(th_es_fd).*(om_es.^2) - cos(th_es_fd).*al_es );
                rddx_sc = -L_sc .* ( cos(th_sc_fd).*(om_sc.^2) + sin(th_sc_fd).*al_sc );
                rddz_sc = -L_sc .* ( sin(th_sc_fd).*(om_sc.^2) - cos(th_sc_fd).*al_sc );
                
                % 누적 전파
                ax_elbow_m    = ax_w + rddx_we;
                az_elbow_m    = az_w + rddz_we;
                ax_shoulder_m = ax_elbow_m + rddx_es;
                az_shoulder_m = az_elbow_m + rddz_es;
                
                % (참값) joint 가속도
                vx_w = gradient(wX, dt); ax_w_t = gradient(vx_w, dt);
                vz_w = gradient(wZ, dt); az_w_t = gradient(vz_w, dt);
                vx_e = gradient(eX, dt); ax_e_t = gradient(vx_e, dt);
                vz_e = gradient(eZ, dt); az_e_t = gradient(vz_e, dt);
                vx_sj = gradient(sX, dt); ax_s_t = gradient(vx_sj, dt);
                vz_sj = gradient(sZ, dt); az_s_t = gradient(vz_sj, dt);
                
                % (참값) 상대가속도
                inc_we_tx = ax_e_t - ax_w_t;   inc_we_tz = az_e_t - az_w_t;
                inc_es_tx = ax_s_t - ax_e_t;   inc_es_tz = az_s_t - az_e_t;
                
                if ~isempty(ax_sac) && ~isempty(az_sac) && numel(ax_sac)==N && numel(az_sac)==N
                    inc_sc_tx = ax_sac - ax_s_t;
                    inc_sc_tz = az_sac - az_s_t;
                    have_sc = true;
                else
                    inc_sc_tx = nan(N,1); inc_sc_tz = nan(N,1);
                    have_sc = false;
                end
                
                % 시간정규화
                phase_src = linspace(0,1,N);
                
                wrx_m = interp1(phase_src, ax_w, phase_tgt, 'linear','extrap');
                wrz_m = interp1(phase_src, az_w, phase_tgt, 'linear','extrap');
                iwx_m = interp1(phase_src, rddx_we, phase_tgt, 'linear','extrap');
                iwz_m = interp1(phase_src, rddz_we, phase_tgt, 'linear','extrap');
                esx_m = interp1(phase_src, rddx_es, phase_tgt, 'linear','extrap');
                esz_m = interp1(phase_src, rddz_es, phase_tgt, 'linear','extrap');
                scx_m = interp1(phase_src, rddx_sc, phase_tgt, 'linear','extrap');
                scz_m = interp1(phase_src, rddz_sc, phase_tgt, 'linear','extrap');
                cmx_m = interp1(phase_src, ax_shoulder_m, phase_tgt, 'linear','extrap');
                cmz_m = interp1(phase_src, az_shoulder_m, phase_tgt, 'linear','extrap');
                
                if have_sc
                    scx_t = interp1(phase_src, ax_sac, phase_tgt, 'linear','extrap');
                    scz_t = interp1(phase_src, az_sac, phase_tgt, 'linear','extrap');
                else
                    scx_t = nan(1,Nnorm); scz_t = nan(1,Nnorm);
                end
                
                wtx = interp1(phase_src, ax_w_t, phase_tgt, 'linear','extrap');
                wtz = interp1(phase_src, az_w_t, phase_tgt, 'linear','extrap');
                etx = interp1(phase_src, ax_e_t, phase_tgt, 'linear','extrap');
                etz = interp1(phase_src, az_e_t, phase_tgt, 'linear','extrap');
                stx = interp1(phase_src, ax_s_t, phase_tgt, 'linear','extrap');
                stz = interp1(phase_src, az_s_t, phase_tgt, 'linear','extrap');
                
                iwx_t = interp1(phase_src, inc_we_tx, phase_tgt, 'linear','extrap');
                iwz_t = interp1(phase_src, inc_we_tz, phase_tgt, 'linear','extrap');
                esx_t = interp1(phase_src, inc_es_tx, phase_tgt, 'linear','extrap');
                esz_t = interp1(phase_src, inc_es_tz, phase_tgt, 'linear','extrap');
                scx_it = interp1(phase_src, inc_sc_tx, phase_tgt, 'linear','extrap');
                scz_it = interp1(phase_src, inc_sc_tz, phase_tgt, 'linear','extrap');
                
                sum_comp_x = wtx + iwx_t + esx_t + scx_it;
                sum_comp_z = wtz + iwz_t + esz_t + scz_it;
                
                % 집계
                M_wrist_x(end+1,:) = wrx_m(:).';   M_wrist_z(end+1,:) = wrz_m(:).';
                M_inc_we_x(end+1,:) = iwx_m(:).';  M_inc_we_z(end+1,:) = iwz_m(:).';
                M_inc_es_x(end+1,:) = esx_m(:).';  M_inc_es_z(end+1,:) = esz_m(:).';
                M_inc_sc_x(end+1,:) = scx_m(:).';  M_inc_sc_z(end+1,:) = scz_m(:).';
                M_cum_x(end+1,:) = cmx_m(:).';     M_cum_z(end+1,:) = cmz_m(:).';
                M_sac_x(end+1,:) = scx_t(:).';     M_sac_z(end+1,:) = scz_t(:).';
                
                M_wrist_true_x(end+1,:) = wtx(:).';  M_wrist_true_z(end+1,:) = wtz(:).';
                M_elbow_true_x(end+1,:) = etx(:).';  M_elbow_true_z(end+1,:) = etz(:).';
                M_shldr_true_x(end+1,:) = stx(:).';  M_shldr_true_z(end+1,:) = stz(:).';
                M_inc_we_true_x(end+1,:) = iwx_t(:).';  M_inc_we_true_z(end+1,:) = iwz_t(:).';
                M_inc_es_true_x(end+1,:) = esx_t(:).';  M_inc_es_true_z(end+1,:) = esz_t(:).';
                M_inc_sc_true_x(end+1,:) = scx_it(:).'; M_inc_sc_true_z(end+1,:) = scz_it(:).';
                
                M_sum_comp_x(end+1,:) = sum_comp_x(:).';
                M_sum_comp_z(end+1,:) = sum_comp_z(:).';
                
                % [추가] Subject / Session 라벨 저장
                M_subj_label{end+1,1} = subj;
                M_sess_label{end+1,1} = sess;   % ★ 추가
                
                n_stride_used = n_stride_used + 1;
                if any(isfinite(scx_t)) && any(isfinite(scz_t)), n_stride_with_sacrum = n_stride_with_sacrum + 1; end
            end
        end
    end
end
fprintf('[INFO] total strides: %d, used: %d, skipped: %d, with-sacrum: %d\n', ...
    n_stride_total, n_stride_used, n_stride_skipped, n_stride_with_sacrum);
% ================================================================
% 통계/분석 — Subject별 RMSE/Corr → mean ± SD across subjects
% ================================================================
% 모델 (플롯용 전체 평균/표준편차 — 기존과 동일)
mu_wx = nanmean(M_wrist_x,1); sd_wx = nanstd(M_wrist_x,0,1);
mu_wz = nanmean(M_wrist_z,1); sd_wz = nanstd(M_wrist_z,0,1);
mu_iwx = nanmean(M_inc_we_x,1); sd_iwx = nanstd(M_inc_we_x,0,1);
mu_iwz = nanmean(M_inc_we_z,1); sd_iwz = nanstd(M_inc_we_z,0,1);
mu_esx = nanmean(M_inc_es_x,1); sd_esx = nanstd(M_inc_es_x,0,1);
mu_esz = nanmean(M_inc_es_z,1); sd_esz = nanstd(M_inc_es_z,0,1);
mu_scincx = nanmean(M_inc_sc_x,1); sd_scincx = nanstd(M_inc_sc_x,0,1);
mu_scincz = nanmean(M_inc_sc_z,1); sd_scincz = nanstd(M_inc_sc_z,0,1);
mu_cmx = nanmean(M_cum_x,1);    sd_cmx = nanstd(M_cum_x,0,1);
mu_cmz = nanmean(M_cum_z,1);    sd_cmz = nanstd(M_cum_z,0,1);
mu_scx = nanmean(M_sac_x,1);    sd_scx = nanstd(M_sac_x,0,1);
mu_scz = nanmean(M_sac_z,1);    sd_scz = nanstd(M_sac_z,0,1);
% 참값 (플롯용)
mu_wtx = nanmean(M_wrist_true_x,1); sd_wtx = nanstd(M_wrist_true_x,0,1);
mu_wtz = nanmean(M_wrist_true_z,1); sd_wtz = nanstd(M_wrist_true_z,0,1);
mu_iwx_t = nanmean(M_inc_we_true_x,1); sd_iwx_t = nanstd(M_inc_we_true_x,0,1);
mu_iwz_t = nanmean(M_inc_we_true_z,1); sd_iwz_t = nanstd(M_inc_we_true_z,0,1);
mu_esx_t = nanmean(M_inc_es_true_x,1); sd_esx_t = nanstd(M_inc_es_true_x,0,1);
mu_esz_t = nanmean(M_inc_es_true_z,1); sd_esz_t = nanstd(M_inc_es_true_z,0,1);
mu_scx_it = nanmean(M_inc_sc_true_x,1); sd_scx_it = nanstd(M_inc_sc_true_x,0,1);
mu_scz_it = nanmean(M_inc_sc_true_z,1); sd_scz_it = nanstd(M_inc_sc_true_z,0,1);
mu_sumx = nanmean(M_sum_comp_x,1); sd_sumx = nanstd(M_sum_comp_x,0,1);
mu_sumz = nanmean(M_sum_comp_z,1); sd_sumz = nanstd(M_sum_comp_z,0,1);
% ================================================================
% [수정] Subject별 RMSE / Corr 계산 → mean ± SD 보고
% ================================================================
vec = @(M) M(:);
unique_subj = unique(M_subj_label, 'stable');
nSubj = numel(unique_subj);
fprintf('\n=== Subject별 통계 (N_subjects = %d) ===\n', nSubj);
% --- 헬퍼 함수: 한 subject의 RMSE, Corr 계산 ---
calc_rmse = @(pred, true_val) sqrt(nanmean((pred(:) - true_val(:)).^2));
calc_corr = @(pred, true_val) corr_pairwise(pred(:), true_val(:));
% Subject별 결과 저장 배열
subj_rmse_we_x  = nan(nSubj,1); subj_rmse_we_z  = nan(nSubj,1);
subj_corr_we_x  = nan(nSubj,1); subj_corr_we_z  = nan(nSubj,1);
subj_rmse_es_x  = nan(nSubj,1); subj_rmse_es_z  = nan(nSubj,1);
subj_corr_es_x  = nan(nSubj,1); subj_corr_es_z  = nan(nSubj,1);
subj_rmse_sc_x  = nan(nSubj,1); subj_rmse_sc_z  = nan(nSubj,1);
subj_corr_sc_x  = nan(nSubj,1); subj_corr_sc_z  = nan(nSubj,1);
subj_rmse_comp_x = nan(nSubj,1); subj_rmse_comp_z = nan(nSubj,1);
subj_corr_comp_x = nan(nSubj,1); subj_corr_comp_z = nan(nSubj,1);
subj_rmse_2seg_x = nan(nSubj,1); subj_rmse_2seg_z = nan(nSubj,1);
subj_rmse_3seg_x = nan(nSubj,1); subj_rmse_3seg_z = nan(nSubj,1);
subj_nrmse_2seg_x = nan(nSubj,1); subj_nrmse_2seg_z = nan(nSubj,1);
subj_nrmse_3seg_x = nan(nSubj,1); subj_nrmse_3seg_z = nan(nSubj,1);
subj_nrmse_3seg_x_half = nan(nSubj,1); subj_nrmse_3seg_z_half = nan(nSubj,1);
subj_n_strides = nan(nSubj,1);
subj_n_sacrum  = nan(nSubj,1);
M_sum_model_x = M_cum_x + M_inc_sc_x;
M_sum_model_z = M_cum_z + M_inc_sc_z;
idx_half = ph_pct <= 50;
for iSub = 1:nSubj
    mask = strcmp(M_subj_label, unique_subj{iSub});
    subj_n_strides(iSub) = sum(mask);
    
    % 해당 subject의 데이터 추출
    pred_we_x = M_inc_we_x(mask,:);   true_we_x = M_inc_we_true_x(mask,:);
    pred_we_z = M_inc_we_z(mask,:);   true_we_z = M_inc_we_true_z(mask,:);
    pred_es_x = M_inc_es_x(mask,:);   true_es_x = M_inc_es_true_x(mask,:);
    pred_es_z = M_inc_es_z(mask,:);   true_es_z = M_inc_es_true_z(mask,:);
    pred_sc_x = M_inc_sc_x(mask,:);   true_sc_x = M_inc_sc_true_x(mask,:);
    pred_sc_z = M_inc_sc_z(mask,:);   true_sc_z = M_inc_sc_true_z(mask,:);
    sac_x_s   = M_sac_x(mask,:);      comp_x_s  = M_sum_comp_x(mask,:);
    sac_z_s   = M_sac_z(mask,:);      comp_z_s  = M_sum_comp_z(mask,:);
    cum_x_s   = M_cum_x(mask,:);
    cum_z_s   = M_cum_z(mask,:);
    sum_model_x_s = M_sum_model_x(mask,:);
    sum_model_z_s = M_sum_model_z(mask,:);
    
    % WE increment
    subj_rmse_we_x(iSub) = calc_rmse(pred_we_x, true_we_x);
    subj_rmse_we_z(iSub) = calc_rmse(pred_we_z, true_we_z);
    subj_corr_we_x(iSub) = calc_corr(pred_we_x, true_we_x);
    subj_corr_we_z(iSub) = calc_corr(pred_we_z, true_we_z);
    
    % ES increment
    subj_rmse_es_x(iSub) = calc_rmse(pred_es_x, true_es_x);
    subj_rmse_es_z(iSub) = calc_rmse(pred_es_z, true_es_z);
    subj_corr_es_x(iSub) = calc_corr(pred_es_x, true_es_x);
    subj_corr_es_z(iSub) = calc_corr(pred_es_z, true_es_z);
    
    % SC increment
    subj_rmse_sc_x(iSub) = calc_rmse(pred_sc_x, true_sc_x);
    subj_rmse_sc_z(iSub) = calc_rmse(pred_sc_z, true_sc_z);
    subj_corr_sc_x(iSub) = calc_corr(pred_sc_x, true_sc_x);
    subj_corr_sc_z(iSub) = calc_corr(pred_sc_z, true_sc_z);
    
    % Sacrum composition (true comp vs model comp)
    subj_rmse_comp_x(iSub) = calc_rmse(sac_x_s, comp_x_s);
    subj_rmse_comp_z(iSub) = calc_rmse(sac_z_s, comp_z_s);
    subj_corr_comp_x(iSub) = calc_corr(sac_x_s, comp_x_s);
    subj_corr_comp_z(iSub) = calc_corr(sac_z_s, comp_z_s);
    
    % sacrum이 있는 stride 수
    subj_n_sacrum(iSub) = sum(any(isfinite(sac_x_s),2) & any(isfinite(sac_z_s),2));
    
    % 2-seg model (W+WE+ES) vs Sacrum
    err_2x = vec(cum_x_s - sac_x_s);
    err_2z = vec(cum_z_s - sac_z_s);
    subj_rmse_2seg_x(iSub) = sqrt(nanmean(err_2x.^2));
    subj_rmse_2seg_z(iSub) = sqrt(nanmean(err_2z.^2));
    
    % 3-seg model (W+WE+ES+SC) vs Sacrum
    err_3x = vec(sum_model_x_s - sac_x_s);
    err_3z = vec(sum_model_z_s - sac_z_s);
    subj_rmse_3seg_x(iSub) = sqrt(nanmean(err_3x.^2));
    subj_rmse_3seg_z(iSub) = sqrt(nanmean(err_3z.^2));
    
    % nRMSE (subject별 range로 정규화)
    sac_x_vec_s = vec(sac_x_s);  sac_z_vec_s = vec(sac_z_s);
    range_x_s = max(sac_x_vec_s,[],'omitnan') - min(sac_x_vec_s,[],'omitnan');
    range_z_s = max(sac_z_vec_s,[],'omitnan') - min(sac_z_vec_s,[],'omitnan');
    if range_x_s == 0, range_x_s = 1; end
    if range_z_s == 0, range_z_s = 1; end
    subj_nrmse_2seg_x(iSub) = (subj_rmse_2seg_x(iSub) / range_x_s) * 100;
    subj_nrmse_2seg_z(iSub) = (subj_rmse_2seg_z(iSub) / range_z_s) * 100;
    subj_nrmse_3seg_x(iSub) = (subj_rmse_3seg_x(iSub) / range_x_s) * 100;
    subj_nrmse_3seg_z(iSub) = (subj_rmse_3seg_z(iSub) / range_z_s) * 100;
    
    % 0~50% 구간 nRMSE (3-seg)
    sac_x_half_s = vec(sac_x_s(:,idx_half));
    model_x_half_s = vec(sum_model_x_s(:,idx_half));
    range_x_half_s = max(sac_x_half_s,[],'omitnan') - min(sac_x_half_s,[],'omitnan');
    if range_x_half_s == 0, range_x_half_s = 1; end
    rmse_x_half_s = sqrt(nanmean((model_x_half_s - sac_x_half_s).^2));
    subj_nrmse_3seg_x_half(iSub) = (rmse_x_half_s / range_x_half_s) * 100;
    
    sac_z_half_s = vec(sac_z_s(:,idx_half));
    model_z_half_s = vec(sum_model_z_s(:,idx_half));
    range_z_half_s = max(sac_z_half_s,[],'omitnan') - min(sac_z_half_s,[],'omitnan');
    if range_z_half_s == 0, range_z_half_s = 1; end
    rmse_z_half_s = sqrt(nanmean((model_z_half_s - sac_z_half_s).^2));
    subj_nrmse_3seg_z_half(iSub) = (rmse_z_half_s / range_z_half_s) * 100;
    
    fprintf('  [%s] strides=%d (sacrum=%d) | WE RMSE X=%.4f Z=%.4f | ES RMSE X=%.4f Z=%.4f | SC RMSE X=%.4f Z=%.4f\n', ...
        unique_subj{iSub}, subj_n_strides(iSub), subj_n_sacrum(iSub), ...
        subj_rmse_we_x(iSub), subj_rmse_we_z(iSub), ...
        subj_rmse_es_x(iSub), subj_rmse_es_z(iSub), ...
        subj_rmse_sc_x(iSub), subj_rmse_sc_z(iSub));
end
% --- Corr: Fisher-z 변환 후 평균 → 역변환 ---
fisherz  = @(r) 0.5 * log((1+r)./(1-r));
ifisherz = @(z) (exp(2*z)-1)./(exp(2*z)+1);
mean_corr = @(r_vec) ifisherz(nanmean(fisherz(r_vec)));
sd_corr   = @(r_vec) nanstd(fisherz(r_vec), 0);  % SD는 z-domain에서 계산
% --- 최종 보고: mean ± SD across subjects ---
fprintf('\n============================================================\n');
fprintf('=== 최종 결과: mean ± SD across %d subjects ===\n', nSubj);
fprintf('============================================================\n');
fprintf('\n--- Segment Increment Accuracy ---\n');
fprintf('[ACC] WE inc — RMSE X: %.4f ± %.4f | Z: %.4f ± %.4f\n', ...
    nanmean(subj_rmse_we_x), nanstd(subj_rmse_we_x,0), ...
    nanmean(subj_rmse_we_z), nanstd(subj_rmse_we_z,0));
fprintf('[ACC] WE inc — Corr X: %.3f (z-SD=%.3f) | Z: %.3f (z-SD=%.3f)\n', ...
    mean_corr(subj_corr_we_x), sd_corr(subj_corr_we_x), ...
    mean_corr(subj_corr_we_z), sd_corr(subj_corr_we_z));
fprintf('[ACC] ES inc — RMSE X: %.4f ± %.4f | Z: %.4f ± %.4f\n', ...
    nanmean(subj_rmse_es_x), nanstd(subj_rmse_es_x,0), ...
    nanmean(subj_rmse_es_z), nanstd(subj_rmse_es_z,0));
fprintf('[ACC] ES inc — Corr X: %.3f (z-SD=%.3f) | Z: %.3f (z-SD=%.3f)\n', ...
    mean_corr(subj_corr_es_x), sd_corr(subj_corr_es_x), ...
    mean_corr(subj_corr_es_z), sd_corr(subj_corr_es_z));
fprintf('[ACC] SC inc — RMSE X: %.4f ± %.4f | Z: %.4f ± %.4f\n', ...
    nanmean(subj_rmse_sc_x), nanstd(subj_rmse_sc_x,0), ...
    nanmean(subj_rmse_sc_z), nanstd(subj_rmse_sc_z,0));
fprintf('[ACC] SC inc — Corr X: %.3f (z-SD=%.3f) | Z: %.3f (z-SD=%.3f)\n', ...
    mean_corr(subj_corr_sc_x), sd_corr(subj_corr_sc_x), ...
    mean_corr(subj_corr_sc_z), sd_corr(subj_corr_sc_z));
fprintf('[ACC] Sacrum comp — RMSE X: %.4f ± %.4f | Z: %.4f ± %.4f\n', ...
    nanmean(subj_rmse_comp_x), nanstd(subj_rmse_comp_x,0), ...
    nanmean(subj_rmse_comp_z), nanstd(subj_rmse_comp_z,0));
fprintf('[ACC] Sacrum comp — Corr X: %.3f (z-SD=%.3f) | Z: %.3f (z-SD=%.3f)\n', ...
    mean_corr(subj_corr_comp_x), sd_corr(subj_corr_comp_x), ...
    mean_corr(subj_corr_comp_z), sd_corr(subj_corr_comp_z));
fprintf('\n--- Model Cumulative vs True Sacrum ---\n');
fprintf('[ERR] (W+WE+ES) vs Sacrum — RMSE X: %.4f ± %.4f | Z: %.4f ± %.4f\n', ...
    nanmean(subj_rmse_2seg_x), nanstd(subj_rmse_2seg_x,0), ...
    nanmean(subj_rmse_2seg_z), nanstd(subj_rmse_2seg_z,0));
fprintf('[ERR] (W+WE+ES) vs Sacrum — nRMSE X: %.2f ± %.2f%% | Z: %.2f ± %.2f%%\n', ...
    nanmean(subj_nrmse_2seg_x), nanstd(subj_nrmse_2seg_x,0), ...
    nanmean(subj_nrmse_2seg_z), nanstd(subj_nrmse_2seg_z,0));
fprintf('[ERR] (W+WE+ES+SC) vs Sacrum — RMSE X: %.4f ± %.4f | Z: %.4f ± %.4f\n', ...
    nanmean(subj_rmse_3seg_x), nanstd(subj_rmse_3seg_x,0), ...
    nanmean(subj_rmse_3seg_z), nanstd(subj_rmse_3seg_z,0));
fprintf('[ERR] (W+WE+ES+SC) vs Sacrum — nRMSE X: %.2f ± %.2f%% | Z: %.2f ± %.2f%%\n', ...
    nanmean(subj_nrmse_3seg_x), nanstd(subj_nrmse_3seg_x,0), ...
    nanmean(subj_nrmse_3seg_z), nanstd(subj_nrmse_3seg_z,0));
fprintf('[0-50%%] (W+WE+ES+SC) nRMSE — X: %.2f ± %.2f%% | Z: %.2f ± %.2f%%\n', ...
    nanmean(subj_nrmse_3seg_x_half), nanstd(subj_nrmse_3seg_x_half,0), ...
    nanmean(subj_nrmse_3seg_z_half), nanstd(subj_nrmse_3seg_z_half,0));
fprintf('============================================================\n\n');
% ================================================================
% 플롯용 추가 변수 (기존 코드 호환)
% ================================================================
mu_sum_model_x = nanmean(M_sum_model_x,1);
sd_sum_model_x = nanstd(M_sum_model_x,0,1);
mu_sum_model_z = nanmean(M_sum_model_z,1);
sd_sum_model_z = nanstd(M_sum_model_z,0,1);
% 색상 정의
col_wrist    = [192 0   0  ]/255;
col_we       = [228 172 41 ]/255;
col_es       = [21  107 177]/255;
col_sc       = [64  162 55 ]/255;
col_sac_est  = [112 48  160]/255;
col_sac_true = [0   0   0  ];
col_sac_sd   = [0.6 0.6 0.6];
lighten = @(c) 0.3 + 0.7*c;

%% Figure 4-1) Segment별 가속도 추정 (Figure plot 파트)

% ----------------------------------------------------------------
SHOW_XLABEL      = false;
SHOW_YLABEL      = false;
DRAW_PHASE_LINES = true;
phase_positions  = [20, 70];
phase_line_color = [0.5 0.5 0.5];
phase_line_style = '--';
phase_line_width = 1.2;

% ----------------------------------------------------------------
%  공통 설정
% ----------------------------------------------------------------
fig_size       = [300 300 180 160];
x_ticks_val    = [0 50 100];
y_ticks        = [-10 0 10];
y_lim          = [-10 10];
font_size      = 8;
linewidth_axis        = 0.8;
linewidth_line        = 1.5;
face_alpha_sd  = 0.25;
face_alpha_est = 0.25;

close all;

% ----------------------------------------------------------------
%  색상 정의
% ----------------------------------------------------------------
col_we       = [153  0   51] / 255;
col_es       = [153  0   51] / 255;
col_sc       = [153  0   51] / 255;
col_com_est  = [153  0   51] / 255;
col_sd_patch = [0.2 0.2 0.2];
lighten      = @(c)  0.7 * c;

% ----------------------------------------------------------------
%  ph_pct / xv 초기화
% ----------------------------------------------------------------
if ~exist('ph_pct', 'var')
    ph_pct = linspace(0, 100, length(mu_iwx));
end
ph_pct = ph_pct(:).';
xv     = [ph_pct, fliplr(ph_pct)];

% ----------------------------------------------------------------
%  공통 스타일 함수
% ----------------------------------------------------------------
apply_style = @(ax) set(ax, ...
    'FontName', 'Helvetica', 'FontSize', font_size, ...
    'Box', 'off', 'TickDir', 'out', 'LineWidth', linewidth_axis, ...
    'XLim', [0 100], 'YLim', y_lim, ...
    'XTick', x_ticks_val, 'YTick', y_ticks, ...
    'XGrid', 'off', 'YGrid', 'off', ...
    'XColor', 'k', 'YColor', 'k');

draw_phase_lines = @(ax) arrayfun(@(x) xline(ax, x, phase_line_style, ...
    'Color', phase_line_color, 'LineWidth', phase_line_width, ...
    'HandleVisibility', 'off'), phase_positions);

% ----------------------------------------------------------------
%  Figure 정의 테이블
%  { Name, mu_true, sd_true, mu_est, sd_est, col_est, ylabel_str }
%  mu_est / sd_est / col_est 가 [] 이면 true 단일 선 그래프
% ----------------------------------------------------------------
figures = {
    'Wrist AP',    mu_wtx,     sd_wtx,     [],             [],              [],          'Wrist Acc_{AP} (m/s^2)';
    'Wrist VT',    mu_wtz,     sd_wtz,     [],             [],              [],          'Wrist Acc_{VT} (m/s^2)';
    'Forearm AP',  mu_iwx_t,   sd_iwx_t,   mu_iwx,         sd_iwx,          col_we,      'Forearm \Delta Acc_{AP} (m/s^2)';
    'Forearm VT',  mu_iwz_t,   sd_iwz_t,   mu_iwz,         sd_iwz,          col_we,      'Forearm \Delta Acc_{VT} (m/s^2)';
    'Upperarm AP', mu_esx_t,   sd_esx_t,   mu_esx,         sd_esx,          col_es,      'Upperarm \Delta Acc_{AP} (m/s^2)';
    'Upperarm VT', mu_esz_t,   sd_esz_t,   mu_esz,         sd_esz,          col_es,      'Upperarm \Delta Acc_{VT} (m/s^2)';
    'Torso AP',    mu_scx_it,  sd_scx_it,  mu_scincx,      sd_scincx,       col_sc,      'Torso \Delta Acc_{AP} (m/s^2)';
    'Torso VT',    mu_scz_it,  sd_scz_it,  mu_scincz,      sd_scincz,       col_sc,      'Torso \Delta Acc_{VT} (m/s^2)';
    'CoM AP',      mu_scx,     sd_scx,     mu_sum_model_x, sd_sum_model_x,  col_com_est, 'CoM Acc_{AP} (m/s^2)';
    'CoM VT',      mu_scz,     sd_scz,     mu_sum_model_z, sd_sum_model_z,  col_com_est, 'CoM Acc_{VT} (m/s^2)';
};

% ----------------------------------------------------------------
%  공통 루프로 Figure 생성
% ----------------------------------------------------------------
for i = 1:size(figures, 1)
    fig_name = figures{i, 1};
    mu_t     = figures{i, 2};
    sd_t     = figures{i, 3};
    mu_e     = figures{i, 4};
    sd_e     = figures{i, 5};
    col_e    = figures{i, 6};
    yl_str   = figures{i, 7};

    figure('Name', fig_name, 'Position', fig_size, 'Color', 'w'); hold on;

    patch(xv, [mu_t - sd_t, fliplr(mu_t + sd_t)], col_sd_patch, ...
        'EdgeColor', 'none', 'FaceAlpha', face_alpha_sd, 'HandleVisibility', 'off');
    plot(ph_pct, mu_t, 'k-', 'LineWidth', linewidth_line);

    if ~isempty(mu_e)
        patch(xv, [mu_e - sd_e, fliplr(mu_e + sd_e)], lighten(col_e), ...
            'EdgeColor', 'none', 'FaceAlpha', face_alpha_est, 'HandleVisibility', 'off');
        plot(ph_pct, mu_e, '-', 'LineWidth', linewidth_line, 'Color', col_e);
    end

    apply_style(gca);
    if DRAW_PHASE_LINES, draw_phase_lines(gca); end
    if SHOW_XLABEL, xlabel('Stride phase (%)'); end
    if SHOW_YLABEL, ylabel(yl_str); end
end

fprintf('전체 Figure 생성 완료. 총 %d개.\n', size(figures, 1));


%%  Table 1) CoM 추정 정확도 표 (LaTeX)
% ========================================================================
%  CoM 추정 정확도 표 (LaTeX)
%  행: Speed (1.0 / 1.25 / 1.5 m/s / Total)  <- (수정) Total Speed 추가
%  열: Phase (Total / Ipsilateral / Contralateral) × (nRMSE, r)
%  AP / VT 각각 출력
%  통계: Subject-level pooled nRMSE, stride별 Corr → subject 평균 → inter-subject Mean ± SD
% ========================================================================

% 0. 변수 확인
req_vars = {'M_sum_model_x','M_sum_model_z','M_sac_x','M_sac_z', ...
            'M_subj_label','M_sess_label','ph_pct'};
% M_sum_model이 없으면 생성 시도
if ~exist('M_sum_model_x','var') && exist('M_cum_x','var') && exist('M_inc_sc_x','var')
    M_sum_model_x = M_cum_x + M_inc_sc_x;
    M_sum_model_z = M_cum_z + M_inc_sc_z;
end
for i = 1:numel(req_vars)
    if ~exist(req_vars{i}, 'var')
        error('필수 변수 누락: %s', req_vars{i});
    end
end

% 1. 설정 (수정: 4번째 세션으로 'All' 추가)
sessions_available = {'ss1','ss2','ss3','All'}; 
speed_labels = {'1.0', '1.25', '1.5', 'Total'};

idx_total  = 1:101;
idx_ipsi   = 1:41;
idx_contra = 51:91;

phases_list = {'Total', 'Ipsilateral', 'Contralateral'};
pidx_list   = {idx_total, idx_ipsi, idx_contra};

axis_names  = {'AP', 'VT', 'Total'};
est_mat     = {M_sum_model_x, M_sum_model_z};
true_mat    = {M_sac_x, M_sac_z};

unique_subj = unique(M_subj_label, 'stable');

% 2. 계산: Session × Axis × Phase → inter-subject nRMSE & Corr
% results{iSe, iAx, iPh} = [nRMSE_mu, nRMSE_sd, Corr_mu, Corr_sd, N]
% iAx: 1=AP, 2=VT, 3=Total(AP+VT 통합)
R = cell(4, 3, 3); % (수정) 행 개수 3 -> 4 로 확장

for iSe = 1:4
    sess = sessions_available{iSe};
    
    % --- AP (iAx=1), VT (iAx=2) ---
    for iAx = 1:2
        E_mat_ax = est_mat{iAx};
        T_mat_ax = true_mat{iAx};
        
        for iPh = 1:3
            pidx = pidx_list{iPh};
            
            subj_nrmse = nan(numel(unique_subj), 1);
            subj_corr  = nan(numel(unique_subj), 1);
            
            for iSub = 1:numel(unique_subj)
                % (수정) iSe == 4 (Total) 일 경우 속도 구분 없이 전체 마스킹
                if iSe == 4
                    mask = strcmp(M_subj_label, unique_subj{iSub});
                else
                    mask = strcmp(M_subj_label, unique_subj{iSub}) & ...
                           strcmp(M_sess_label, sess);
                end
                
                if sum(mask) == 0, continue; end
                
                E_sub = E_mat_ax(mask, pidx);
                T_sub = T_mat_ax(mask, pidx);
                
                e_vec = E_sub(:);
                t_vec = T_sub(:);
                valid = isfinite(e_vec) & isfinite(t_vec);
                if sum(valid) < 10, continue; end
                e_v = e_vec(valid);
                t_v = t_vec(valid);
                
                rmse_val = sqrt(mean((e_v - t_v).^2));
                r_range = max(t_v) - min(t_v);
                if r_range < 0.01, continue; end
                subj_nrmse(iSub) = (rmse_val / r_range) * 100;
                
                nS = size(E_sub, 1);
                corr_arr = nan(nS, 1);
                for ss = 1:nS
                    e_s = E_sub(ss,:);
                    t_s = T_sub(ss,:);
                    vv = isfinite(e_s) & isfinite(t_s);
                    if sum(vv) < 5, continue; end
                    Rc = corrcoef(e_s(vv), t_s(vv));
                    corr_arr(ss) = Rc(1,2);
                end
                subj_corr(iSub) = mean(corr_arr, 'omitnan');
            end
            
            vn = isfinite(subj_nrmse);
            R{iSe, iAx, iPh} = [mean(subj_nrmse,'omitnan'), std(subj_nrmse,0,'omitnan'), ...
                                 mean(subj_corr,'omitnan'),   std(subj_corr,0,'omitnan'), ...
                                 sum(vn)];
        end
    end
    
    % --- Total (iAx=3): AP + VT 통합 ---
    for iPh = 1:3
        pidx = pidx_list{iPh};
        
        subj_nrmse = nan(numel(unique_subj), 1);
        subj_corr  = nan(numel(unique_subj), 1);
        
        for iSub = 1:numel(unique_subj)
            % (수정) iSe == 4 (Total) 마스킹 처리
            if iSe == 4
                mask = strcmp(M_subj_label, unique_subj{iSub});
            else
                mask = strcmp(M_subj_label, unique_subj{iSub}) & ...
                       strcmp(M_sess_label, sess);
            end
            if sum(mask) == 0, continue; end
            
            % AP + VT concat
            E_sub_x = est_mat{1}(mask, pidx);
            T_sub_x = true_mat{1}(mask, pidx);
            E_sub_z = est_mat{2}(mask, pidx);
            T_sub_z = true_mat{2}(mask, pidx);
            
            e_vec = [E_sub_x(:); E_sub_z(:)];
            t_vec = [T_sub_x(:); T_sub_z(:)];
            valid = isfinite(e_vec) & isfinite(t_vec);
            if sum(valid) < 10, continue; end
            e_v = e_vec(valid);
            t_v = t_vec(valid);
            
            rmse_val = sqrt(mean((e_v - t_v).^2));
            r_range = max(t_v) - min(t_v);
            if r_range < 0.01, continue; end
            subj_nrmse(iSub) = (rmse_val / r_range) * 100;
            
            % Corr: stride별, AP와 VT 각각 → 전체 평균
            nS = sum(mask);
            corr_arr = nan(nS * 2, 1);
            E_sx = est_mat{1}(mask, pidx);
            T_sx = true_mat{1}(mask, pidx);
            E_sz = est_mat{2}(mask, pidx);
            T_sz = true_mat{2}(mask, pidx);
            for ss = 1:nS
                % AP
                e_s = E_sx(ss,:); t_s = T_sx(ss,:);
                vv = isfinite(e_s) & isfinite(t_s);
                if sum(vv) >= 5
                    Rc = corrcoef(e_s(vv), t_s(vv));
                    corr_arr(ss) = Rc(1,2);
                end
                % VT
                e_s = E_sz(ss,:); t_s = T_sz(ss,:);
                vv = isfinite(e_s) & isfinite(t_s);
                if sum(vv) >= 5
                    Rc = corrcoef(e_s(vv), t_s(vv));
                    corr_arr(nS + ss) = Rc(1,2);
                end
            end
            subj_corr(iSub) = mean(corr_arr, 'omitnan');
        end
        
        vn = isfinite(subj_nrmse);
        R{iSe, 3, iPh} = [mean(subj_nrmse,'omitnan'), std(subj_nrmse,0,'omitnan'), ...
                           mean(subj_corr,'omitnan'),   std(subj_corr,0,'omitnan'), ...
                           sum(vn)];
    end
end

% 3. 콘솔 출력 (수정: 4개 행 출력)
fprintf('\n================================================================\n');
fprintf('  CoM Acceleration Estimation Accuracy\n');
fprintf('  Subject-level Mean ± SD\n');
fprintf('================================================================\n');

for iAx = 1:3
    fprintf('\n--- %s ---\n', axis_names{iAx});
    fprintf('%-8s', 'Speed');
    for iPh = 1:3
        fprintf('  %22s', phases_list{iPh});
    end
    fprintf('\n%s\n', repmat('-', 1, 78));
    
    for iSe = 1:4
        if iSe == 4
            fprintf('%-8s', speed_labels{iSe}); % 'Total' 은 단위(m/s) 제외
        else
            fprintf('%-8s', [speed_labels{iSe} ' m/s']);
        end
        
        for iPh = 1:3
            v = R{iSe, iAx, iPh};
            fprintf('  %5.1f±%4.1f / %5.2f±%4.2f', v(1), v(2), v(3), v(4));
        end
        fprintf('\n');
    end
end


%% Figure 5, 7) 속도별 Raw data
% [수정] 1명이면 stride-level SD, 여러 명이면 inter-subject SD
clc; close all;
%% Figure 8 → Figure 5-2 스타일) 3개 모델을 한 figure에 (Standard=실선, Speed=점선)
clc; close all;

% =========================================================================
% [0. 기본 설정]
% =========================================================================
base_dir = "D:\바탕화면\데이터\Arm_Swing_Paper";
models       = ["CNN", "Transformer", "Physres_no_physics"];
model_labels = ["CNN", "Transformer", "Proposed"];

% ============================================================
%  [사용자 설정]
% ============================================================
COLOR_OPTION   = 'A';
SHOW_XLABEL    = false;
SHOW_YLABEL    = false;
use_real_scale = false;

% ============================================================
%  색상 옵션
% ============================================================
switch COLOR_OPTION
    case 'A'
        model_colors = [ ...
            153/255   0/255 51/255; ...   % CNN (빨강)
            0/255  102/255 0/255; ...   % Transformer (초록)
            51/255  51/255 153/255; ];    % Proposed (파랑)
    case 'B'
        model_colors = [ ...
            0.8784  0.4824  0.2235; ...
            0.4824  0.3686  0.6549; ...
            0.0000  0.3137  0.6196; ];
    case 'C'
        model_colors = [ ...
            0.4980  0.5490  0.5529; ...
            0.1020  0.5647  0.5647; ...
            0.0000  0.3137  0.6196; ];
end
model_markers = ['o', 'o', 'o'];

% =========================================================================
% [★ 중요 ★] Y-Limit 및 Y-Tick 설정 (Standard + Speed 통합 범위)
% =========================================================================
% --- nRMSE ---
ylim_nrmse_merged  = [0.05, 0.17];
ytick_nrmse_merged = 0.05 : 0.03 : 0.17;

% --- Correlation ---
ylim_corr_merged   = [0.82, 0.98];
ytick_corr_merged  = 0.82 : 0.04 : 0.98;

% ============================================================
%  Figure / 스타일 설정
% ============================================================
FONT_NAME      = 'Helvetica';
FONT_SIZE      = 10;
linewidth_axis = 1.1;
linewidth_line = 1.7;
MARKER_SZ      = 4.5;

% =========================================================================
% [1. 분석 루프]
% =========================================================================
metrics_to_plot = ["nRMSE", "Correlation"];

for met_idx = 1:numel(metrics_to_plot)
    current_metric = metrics_to_plot(met_idx);

    fprintf('\n=======================================================\n');
    fprintf(' Processing Metric: %s\n', current_metric);
    fprintf('=======================================================\n');

    % -----------------------------------------------------------------
    %  설정 정의 (A: Standard / C: Speed)
    % -----------------------------------------------------------------
    x_labels_common = ["5", "10", "20", "50", "100"];

    % Config A: Standard LOSO
    configA.id = "A"; configA.title = "Standard LOSO";
    configA.x_labels = x_labels_common; configA.mode = "standard";
    configA.folders = [ ...
        "Figure5_datasize", "5percent";
        "Figure5_datasize", "10percent";
        "Figure5_datasize", "20percent";
        "Figure5_datasize", "50percent";
        "Figure3_loso",     ""          ];

    % Config C: Speed LOSO
    configC.id = "C"; configC.title = "Speed LOSO";
    configC.x_labels = x_labels_common; configC.mode = "speed";
    configC.folders = [ ...
        "Figure6_datasize_sploso", "5percent";
        "Figure6_datasize_sploso", "10percent";
        "Figure6_datasize_sploso", "20percent";
        "Figure6_datasize_sploso", "50percent";
        "Figure4_speedloso",       ""          ];

    % -----------------------------------------------------------------
    %  데이터 로드
    % -----------------------------------------------------------------
    dataA = loadFigData(configA, models, base_dir, current_metric);
    dataC = loadFigData(configC, models, base_dir, current_metric);

    % -----------------------------------------------------------------
    %  범위 및 라벨 설정
    % -----------------------------------------------------------------
    if current_metric == "nRMSE"
        y_label_text = "nRMSE";
        curr_ylim  = ylim_nrmse_merged;
        curr_ytick = ytick_nrmse_merged;
    else
        y_label_text = "Correlation (R)";
        curr_ylim  = ylim_corr_merged;
        curr_ytick = ytick_corr_merged;
    end

    % -----------------------------------------------------------------
    %  그래프 그리기 (Overlay: 실선=Standard, 점선=Speed)
    % -----------------------------------------------------------------
    figure('Color', 'w', 'Position', [100, 100, 330, 280], ...
           'Name', sprintf('Overlay Plot [%s]', current_metric));
    ax = axes();
    hold(ax, 'on');  box(ax, 'on');

    n_models = numel(models);
    n_points = numel(x_labels_common);
    x_axis   = 1:n_points;

    legend_handles = [];

    for m = 1:n_models
        % 1. Standard LOSO → 실선(-)  마커 채움
        h1 = plot(ax, x_axis, dataA(m, :), ...
            'LineStyle',       ':', ...
            'LineWidth',       linewidth_line, ...
            'Marker',          model_markers(m), ...
            'MarkerSize',      MARKER_SZ, ...
            'Color',           model_colors(m, :), ...
            'MarkerFaceColor', model_colors(m, :), ...
            'DisplayName',     sprintf('%s (Sub)', model_labels(m)));

        % 2. Speed LOSO → 점선(--)  마커 내부 흰색
        h2 = plot(ax, x_axis, dataC(m, :), ...
            'LineStyle',       '-', ...
            'LineWidth',       linewidth_line, ...
            'Marker',          model_markers(m), ...
            'MarkerSize',      MARKER_SZ, ...
            'Color',           model_colors(m, :), ...
            'MarkerFaceColor', 'w', ...
            'DisplayName',     sprintf('%s (Spd)', model_labels(m)));

        legend_handles = [legend_handles, h1, h2]; %#ok<AGROW>
    end

    % --- 축 스타일 ---
    set(ax, 'XTick', x_axis, 'XTickLabel', x_labels_common);
    set(ax, 'YTick', curr_ytick);
    ylim(ax, curr_ylim);
    xlim(ax, [0.5, 5.5]);

    set(ax, 'FontSize', FONT_SIZE, 'FontName', FONT_NAME, ...
            'FontWeight', 'bold', ...
            'Box', 'off', 'TickDir', 'out', 'LineWidth', linewidth_axis);

    if SHOW_XLABEL
        xlabel(ax, 'Dataset size (%)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    end
    if SHOW_YLABEL
        ylabel(ax, y_label_text, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    end

    % 범례 (2열 배치)
    % legend(ax, legend_handles, ...
    %     'Location', 'best', 'FontSize', 7, 'Box', 'on', 'NumColumns', 2);
    % 
    % hold(ax, 'off');

    % -----------------------------------------------------------------
    %  콘솔 출력
    % -----------------------------------------------------------------
    fprintf('\n  [%s] Standard LOSO:\n', current_metric);
    fprintf('  %-25s', 'Model');
    for x = 1:numel(x_labels_common), fprintf('%10s%%', x_labels_common(x)); end
    fprintf('\n  %s\n', repmat('-', 1, 75));
    for m = 1:n_models
        fprintf('  %-25s', model_labels(m));
        for x = 1:numel(x_labels_common), fprintf('%11.4f', dataA(m, x)); end
        fprintf('\n');
    end

    fprintf('\n  [%s] Speed LOSO:\n', current_metric);
    fprintf('  %-25s', 'Model');
    for x = 1:numel(x_labels_common), fprintf('%10s%%', x_labels_common(x)); end
    fprintf('\n  %s\n', repmat('-', 1, 75));
    for m = 1:n_models
        fprintf('  %-25s', model_labels(m));
        for x = 1:numel(x_labels_common), fprintf('%11.4f', dataC(m, x)); end
        fprintf('\n');
    end
end

fprintf('\n모든 그래프 작성이 완료되었습니다.\n');



%% Table 2) AP vs Vertical according to speed
% 1. nRMSE: x100 후 소수점 둘째 자리 (예: 5.12)
% 2. Correlation: 소수점 둘째 자리 (예: 0.95)
% 3. 순서:  PhysResNet
% 4. 속도: Slow -> Moderate -> Fast -> Average

clc; close all;

% =========================================================================
% [1. 기본 설정]
% =========================================================================
base_root = "D:\바탕화면\데이터\Arm_Swing_Paper"; % 데이터 최상위 경로
models    = ["Physres_no_physics"];    % 실제 폴더명
disp_names = ["PhysResNet"]; % 출력용 모델명

% 파일 및 라벨 설정
% ss1: Slow, ss2: Moderate, ss3: Fast, ss123: Average (Overall)
file_list  = ["errorTable_ss1.mat", "errorTable_ss2.mat", "errorTable_ss3.mat", "errorTable_ss123.mat"];
row_labels = ["Slow", "Moderate", "Fast", "Total"];

fprintf('\n%% =======================================================\n');
fprintf('%% [Start] Copy below for LaTeX Table Body\n');
fprintf('%% Columns: Condition & nRMSE(Total) & nRMSE(AP) & nRMSE(VT) & Corr(Total) & Corr(AP) & Corr(VT) \\\\\n');
fprintf('%% =======================================================\n');

% =========================================================================
% [2. 모델별 루프]
% =========================================================================
for m_idx = 1:numel(models)
    curr_model = models(m_idx);
    curr_disp  = disp_names(m_idx);
    
    % LaTeX 주석으로 모델 구분
    fprintf('\n%% --- Model: %s ---\n', curr_disp);
    
    % 모델이름을 첫 행에 멀티컬럼으로 넣을지, 아니면 그냥 행으로 나열할지 선택
    % 여기서는 사용자 요청대로 행(Row) 단위로 출력합니다.
    % 만약 모델명을 표 안에 넣고 싶다면 \multirow 등을 사용해야 합니다.
    % 여기서는 단순히 구분용 주석만 남기고 데이터 행을 출력합니다.
    
    model_dir = fullfile(base_root, curr_model, "Figure3_loso");
    
    % ---------------------------------------------------------------------
    % [3. 속도 조건별 루프 (Row)]
    % ---------------------------------------------------------------------
    for f_idx = 1:numel(file_list)
        fname = file_list(f_idx);
        fpath = fullfile(model_dir, fname);
        r_label = row_labels(f_idx);
        
        % 값 초기화
        v_nrmse = [NaN, NaN, NaN]; s_nrmse = [NaN, NaN, NaN];
        v_corr  = [NaN, NaN, NaN]; s_corr  = [NaN, NaN, NaN];
        
        if isfile(fpath)
            try
                S = load(fpath);
                vars = fieldnames(S);
                T = S.(vars{1});
                
                % 첫 번째 행 (Total Subject Mean) 추출
                row_d = T(1, :);
                
                % [변수명 매핑 확인]
                % 이전 코드들에서 Fx, Fz로 저장되었으므로 이를 우선 사용
                % 만약 GRFx라면 catch문이나 if문으로 처리 가능하지만, 여기선 Fx/Fz로 통일
                
                % 1) nRMSE (Total, AP, VT)
                % AP: Fx, VT: Fz 라고 가정
                v_nrmse = [row_d.Mean_nRMSE_all, row_d.Mean_nRMSE_Fx, row_d.Mean_nRMSE_Fz];
                s_nrmse = [row_d.Std_nRMSE_all,  row_d.Std_nRMSE_Fx,  row_d.Std_nRMSE_Fz];
                
                % 2) Correlation (Total, AP, VT)
                v_corr  = [row_d.Mean_Corr_all, row_d.Mean_Corr_Fx, row_d.Mean_Corr_Fz];
                s_corr  = [row_d.Std_Corr_all,  row_d.Std_Corr_Fx,  row_d.Std_Corr_Fz];
                
            catch
                % 변수명 불일치 등의 에러 처리
                fprintf('%% [Error parsing %s] Check variable names (Fx vs GRFx)\n', fname);
                continue;
            end
        else
             fprintf('%% [File not found: %s]\n', fname);
             continue;
        end
        
        % -----------------------------------------------------------------
        % [4. 텍스트 포맷팅 (LaTeX)]
        % -----------------------------------------------------------------
        % nRMSE: % 단위 변환 (*100)
        n_str_tot = sprintf('%.2f \\pm %.2f', v_nrmse(1)*100, s_nrmse(1)*100);
        n_str_ap  = sprintf('%.2f \\pm %.2f', v_nrmse(2)*100, s_nrmse(2)*100);
        n_str_vt  = sprintf('%.2f \\pm %.2f', v_nrmse(3)*100, s_nrmse(3)*100);
        
        % Correlation: 소수점 2자리
        c_str_tot = sprintf('%.2f \\pm %.2f', v_corr(1), s_corr(1));
        c_str_ap  = sprintf('%.2f \\pm %.2f', v_corr(2), s_corr(2));
        c_str_vt  = sprintf('%.2f \\pm %.2f', v_corr(3), s_corr(3));
        
        % [출력]
        % 예: ModelName & Slow & 5.12 \pm 1.00 & ... \\
        % 첫 번째 줄(Slow)에만 모델명을 넣고 싶다면 로직 추가 필요.
        % 여기서는 깔끔하게 모든 행에 모델명을 넣거나, 첫 컬럼을 비워두는 방식을 씁니다.
        
        % (Format: Model & Condition & nRMSE(Tot) & (AP) & (VT) & Corr(Tot) & (AP) & (VT) \\)
        if f_idx == 1
            % 첫 번째 행(Slow)에 모델명 표시
            model_col = sprintf('\\multirow{4}{*}{%s}', curr_disp); 
        else
            model_col = ''; 
        end
        
        % 테이블 행 출력
        fprintf('%s & %s & $%s$ & $%s$ & $%s$ & $%s$ & $%s$ & $%s$ \\\\\n', ...
            model_col, r_label, ...
            n_str_tot, n_str_ap, n_str_vt, ...
            c_str_tot, c_str_ap, c_str_vt);
            
        % 그룹 간 구분선 (Average 뒤에)
        if f_idx == 4
             fprintf('\\hline\n');
        end
    end
end

fprintf('%% =======================================================\n');
fprintf('%% [End] Copy above\n');


% ========================================================================
% [5. 속도 간 통계적 유의성 검증 및 피험자별 데이터 출력]
% ========================================================================

fprintf('\n\n%% =======================================================\n');
fprintf('%% [Statistical Test] Speed Effect & Individual Subject Data\n');
fprintf('%% =======================================================\n');

speed_files  = ["errorTable_ss1.mat", "errorTable_ss2.mat", "errorTable_ss3.mat"];
speed_labels = {'Slow','Moderate','Fast'};
pair_names   = {'Slow vs Mod', 'Slow vs Fast', 'Mod vs Fast'};
pair_idx     = [1 2; 1 3; 2 3];  % 3쌍
n_pairs      = 3;

% errorTable에서 피험자별 값이 있는 컬럼 (Mean_ 접두사)
test_metrics = {
    'Mean_nRMSE_all', 'nRMSE(Total)';
    'Mean_nRMSE_Fx',  'nRMSE(AP)';
    'Mean_nRMSE_Fz',  'nRMSE(VT)';
    'Mean_Corr_all',  'Corr(Total)';
    'Mean_Corr_Fx',   'Corr(AP)';
    'Mean_Corr_Fz',   'Corr(VT)'};

for m_idx = 1:numel(models)
    curr_model = models(m_idx);
    curr_disp  = disp_names(m_idx);
    model_dir  = fullfile(base_root, curr_model, "Figure3_loso");
    
    fprintf('\n========== %s ==========\n', curr_disp);
    
    % --- 3속도 errorTable 로드 ---
    ET = cell(1,3);
    load_ok = true;
    for sp = 1:3
        fpath = fullfile(model_dir, speed_files(sp));
        if ~isfile(fpath)
            fprintf('  [Error] %s 없음\n', speed_files(sp));
            load_ok = false; break;
        end
        S = load(fpath); fn = fieldnames(S);
        ET{sp} = S.(fn{1});
    end
    if ~load_ok, continue; end
    
    % --- 피험자 매칭 (1행=Total → 2행부터 개별 피험자) ---
    subj1 = ET{1}.RowLabel(2:end);
    subj2 = ET{2}.RowLabel(2:end);
    subj3 = ET{3}.RowLabel(2:end);
    common_subj = intersect(intersect(subj1, subj2), subj3);
    nSubj = numel(common_subj);
    
    if nSubj < 3
        fprintf('  공통 피험자 %d명 → 검정 불가\n', nSubj);
        continue;
    end
    fprintf('  공통 피험자: %d명\n\n', nSubj);
    
    % 인덱스 매핑
    idx = zeros(nSubj, 3);
    for sp = 1:3
        for j = 1:nSubj
            idx(j, sp) = find(strcmp(ET{sp}.RowLabel, common_subj(j)));
        end
    end
    
    % --- [검정 결과 출력] ---
    fprintf('  [Paired t-test Results]\n');
    fprintf('  %-16s', 'Metric');
    for pp = 1:n_pairs
        fprintf('  | %-22s', pair_names{pp});
    end
    fprintf('\n  %s\n', repmat('-', 1, 85));
    
    for t_idx = 1:size(test_metrics, 1)
        col_name  = test_metrics{t_idx, 1};
        disp_name = test_metrics{t_idx, 2};
        
        V = zeros(nSubj, 3);
        for sp = 1:3
            V(:, sp) = ET{sp}.(col_name)(idx(:, sp));
        end
        
        valid = all(isfinite(V), 2);
        V = V(valid, :);
        n = size(V, 1);
        if n < 3, continue; end
        
        p_raw = NaN(1, n_pairs);
        for pp = 1:n_pairs
            [~, p_raw(pp)] = ttest(V(:, pair_idx(pp,1)), V(:, pair_idx(pp,2)));
        end
        p_bonf = min(p_raw * n_pairs, 1);
        
        fprintf('  %-16s', disp_name);
        for pp = 1:n_pairs
            a_m = mean(V(:, pair_idx(pp,1)), 'omitnan');
            b_m = mean(V(:, pair_idx(pp,2)), 'omitnan');
            if a_m > b_m, dir_ch = '>'; else, dir_ch = '<'; end
            
            if     p_bonf(pp) < 0.001, sig = '***';
            elseif p_bonf(pp) < 0.01,  sig = '**';
            elseif p_bonf(pp) < 0.05,  sig = '*';
            else,                       sig = 'n.s.';
            end
            
            fprintf('  | p=%.4f (%s) %s %s', p_bonf(pp), sig, speed_labels{pair_idx(pp,1)}(1), dir_ch);
            fprintf('%s', speed_labels{pair_idx(pp,2)}(1));
        end
        fprintf('\n');
    end
    
    % --- [추가됨] 피험자별 데이터 출력 ---
    fprintf('\n  [Individual Subject Data]\n');
    
    for t_idx = 1:size(test_metrics, 1)
        col_name  = test_metrics{t_idx, 1};
        disp_name = test_metrics{t_idx, 2};
        
        fprintf('\n  >> %s\n', disp_name);
        fprintf('  %-10s  %-12s  %-12s  %-12s\n', 'Subject', 'Slow', 'Moderate', 'Fast');
        fprintf('  %s\n', repmat('-', 1, 50));
        
        V = zeros(nSubj, 3);
        for sp = 1:3
            V(:, sp) = ET{sp}.(col_name)(idx(:, sp));
        end
        
        % nRMSE는 x100
        is_nrmse = contains(col_name, 'nRMSE');
        if is_nrmse, V = V * 100; end
        
        % 피험자별 출력 루프
        for i = 1:nSubj
            fprintf('  %-10s', common_subj{i});
            for sp = 1:3
                val = V(i, sp);
                if is_nrmse
                    fprintf('  %10.2f  ', val);
                else
                    fprintf('  %10.4f  ', val);
                end
            end
            fprintf('\n');
        end
        
        % 평균 출력
        fprintf('  %s\n', repmat('-', 1, 50));
        fprintf('  %-10s', 'Mean');
        for sp = 1:3
            fprintf('  %10.4f  ', mean(V(:,sp), 'omitnan'));
        end
        fprintf('\n  %-10s', 'SD');
        for sp = 1:3
            fprintf('  %10.4f  ', std(V(:,sp), 'omitnan'));
        end
        fprintf('\n');
    end
end

fprintf('\n%% =======================================================\n');
fprintf('%% [End] Statistical Tests\n');
fprintf('%% =======================================================\n');
%% Table 3) Ipsilateral vs Contralateral
% 1. Speed LOSO: ss1, ss2, ss3 파일을 찾아 수치(Mean, Std)를 평균내어 하나의 행으로 출력
% 2. Standard LOSO: ss123 파일을 찾아 하나의 행으로 출력
% 3. Output: LaTeX Format

clc; close all;

% =========================================================================
% [1. 기본 설정]
% =========================================================================
base_root = "D:\바탕화면\데이터\Arm_Swing_Paper"; 
target_model = "Physres_no_physics"; 

% 분석할 비율 리스트
ratio_list = [100, 50, 20, 10, 5];

% 파일명 패턴 (Speed LOSO용)
speed_patterns = ["*train_ss1*.mat", "*train_ss2*.mat", "*train_ss3*.mat"];
% 파일명 패턴 (Standard LOSO용)
std_patterns   = ["*ss123*.mat", "*loso*.mat"];

fprintf('\n%% =======================================================\n');
fprintf('%% [Start] Copy below for LaTeX Table Body\n');
fprintf('%% Structure: Dataset & Method & Ipsi & Contra & Flex & Ext \\\\\n');
fprintf('%% =======================================================\n');

% =========================================================================
% [2. 루프 실행: 데이터 크기별]
% =========================================================================
for r_idx = 1:length(ratio_list)
    curr_ratio = ratio_list(r_idx);
    
    % ---------------------------------------------------------------------
    % [Step 1] 폴더 경로 설정
    % ---------------------------------------------------------------------
    if curr_ratio == 100
        path_loso_dir  = fullfile(base_root, target_model, "Figure3_loso");
        path_speed_dir = fullfile(base_root, target_model, "Figure4_speedloso");
    else
        sub_folder_name = sprintf("%dpercent", curr_ratio);
        path_loso_dir  = fullfile(base_root, target_model, "Figure5_datasize", sub_folder_name);
        path_speed_dir = fullfile(base_root, target_model, "Figure6_datasize_sploso", sub_folder_name);
    end
    
    % 구분선 주석
    fprintf('%% --- Dataset Size: %d%% ---\n', curr_ratio);
    
    % =====================================================================
    % [Step 2] Speed LOSO 처리 (ss1, ss2, ss3 평균)
    % =====================================================================
    % 누적 변수 초기화 (nRMSE, Corr / Mean, Std)
    % 순서: [Ipsi, Contra, Flex, Ext]
    sum_n_mu = zeros(1, 4); sum_n_sd = zeros(1, 4);
    sum_c_mu = zeros(1, 4); sum_c_sd = zeros(1, 4);
    count_speed = 0;
    
    for i = 1:3
        % 파일 검색
        found = dir(fullfile(path_speed_dir, speed_patterns(i)));
        if ~isempty(found)
            fpath = fullfile(path_speed_dir, found(1).name);
            [n_mu, n_sd, c_mu, c_sd] = extract_raw_values(fpath);
            
            if ~any(isnan(n_mu))
                sum_n_mu = sum_n_mu + n_mu;
                sum_n_sd = sum_n_sd + n_sd;
                sum_c_mu = sum_c_mu + c_mu;
                sum_c_sd = sum_c_sd + c_sd;
                count_speed = count_speed + 1;
            end
        end
    end
    
    % Speed LOSO 결과 출력 (3개 파일 평균)
    if count_speed > 0
        % 평균 계산
        avg_n_mu = sum_n_mu / count_speed;
        avg_n_sd = sum_n_sd / count_speed;
        avg_c_mu = sum_c_mu / count_speed;
        avg_c_sd = sum_c_sd / count_speed;
        
        % LaTeX String 생성
        % 첫 번째 컬럼에 Dataset Size 표시 (Multirow 처리는 LaTeX에서 하거나 빈칸)
        if r_idx == 1
             ds_str = sprintf('\\multirow{6}{*}{%d\\%%}', curr_ratio); % 예시
        else
             ds_str = sprintf('%d\\%%', curr_ratio);
        end
        
        % nRMSE 행 출력
        fprintf('%s & Speed LOSO (nRMSE) & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ \\\\\n', ...
            ds_str, ...
            avg_n_mu(1), avg_n_sd(1), avg_n_mu(2), avg_n_sd(2), ...
            avg_n_mu(3), avg_n_sd(3), avg_n_mu(4), avg_n_sd(4));
            
        % Correlation 행 출력 (필요하다면) - 여기서는 nRMSE/Corr를 한 줄에 다 넣을지, 행을 나눌지 결정
        % 요청하신 이전 포맷(nRMSE / Corr)을 감안하여 Corr도 출력
        fprintf('%s & Speed LOSO (Corr)  & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ \\\\\n', ...
            '', ... % 첫 컬럼 비움
            avg_c_mu(1), avg_c_sd(1), avg_c_mu(2), avg_c_sd(2), ...
            avg_c_mu(3), avg_c_sd(3), avg_c_mu(4), avg_c_sd(4));
            
    else
        fprintf('%% [Error] Speed files missing in %s\n', path_speed_dir);
    end

    % =====================================================================
    % [Step 3] Standard LOSO 처리 (ss123 단일 파일)
    % =====================================================================
    found_loso = [];
    for p = 1:length(std_patterns)
        found_loso = dir(fullfile(path_loso_dir, std_patterns(p)));
        if ~isempty(found_loso), break; end
    end
    
    if ~isempty(found_loso)
        fpath = fullfile(path_loso_dir, found_loso(1).name);
        [n_mu, n_sd, c_mu, c_sd] = extract_raw_values(fpath);
        
        if ~any(isnan(n_mu))
            % nRMSE 출력
            fprintf('%s & Standard LOSO (nRMSE) & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ \\\\\n', ...
                '', ...
                n_mu(1), n_sd(1), n_mu(2), n_sd(2), n_mu(3), n_sd(3), n_mu(4), n_sd(4));
            
            % Corr 출력
            fprintf('%s & Standard LOSO (Corr)  & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ & $%.2f \\pm %.2f$ \\\\\n', ...
                '', ...
                c_mu(1), c_sd(1), c_mu(2), c_sd(2), c_mu(3), c_sd(3), c_mu(4), c_sd(4));
        end
    else
        fprintf('%% [Error] Standard LOSO file missing in %s\n', path_loso_dir);
    end
    
    % 줄바꿈 (LaTeX)
    fprintf('\\hline\n');
end

fprintf('%% =======================================================\n');
fprintf('%% [End]\n');

% ========================================================================
% [Statistical Test] Contra - Ipsi Error Difference
% 가설 1: Data↓ → Δ(Contra-Ipsi)↑
% 가설 2: Speed LOSO → Δ(Contra-Ipsi) > Standard LOSO
% ========================================================================
fprintf('\n%% =======================================================\n');
fprintf('%% [Statistical Test] Contralateral - Ipsilateral Error Gap\n');
fprintf('%% =======================================================\n');

base_root    = "D:\바탕화면\데이터\Arm_Swing_Paper";
target_model = "Physres_no_physics";
ratio_list   = [100, 50, 20, 10, 5];
speed_patterns = ["*train_ss1*.mat", "*train_ss2*.mat", "*train_ss3*.mat"];
std_patterns   = ["*ss123*.mat", "*loso*.mat"];

delta_nrmse_speed = cell(1, numel(ratio_list));
delta_nrmse_std   = cell(1, numel(ratio_list));
delta_corr_speed  = cell(1, numel(ratio_list));
delta_corr_std    = cell(1, numel(ratio_list));

for r_idx = 1:numel(ratio_list)
    curr_ratio = ratio_list(r_idx);
    
    if curr_ratio == 100
        path_loso_dir  = fullfile(base_root, target_model, "Figure3_loso");
        path_speed_dir = fullfile(base_root, target_model, "Figure4_speedloso");
    else
        sub_folder = sprintf("%dpercent", curr_ratio);
        path_loso_dir  = fullfile(base_root, target_model, "Figure5_datasize", sub_folder);
        path_speed_dir = fullfile(base_root, target_model, "Figure6_datasize_sploso", sub_folder);
    end
    
    % =================================================================
    % Speed LOSO: ss1,ss2,ss3 → 공통 피험자 매칭 후 평균
    % =================================================================
    sp_tables = {};
    sp_subjs  = {};
    for i = 1:3
        found = dir(fullfile(path_speed_dir, speed_patterns(i)));
        if isempty(found), continue; end
        fpath = fullfile(path_speed_dir, found(1).name);
        S = load(fpath); fn = fieldnames(S); T = S.(fn{1});
        sp_tables{end+1} = T;
        sp_subjs{end+1}  = T.RowLabel(2:end);  % 1행=Total 제외
    end
    
    if numel(sp_tables) >= 1
        % 공통 피험자 찾기
        common = sp_subjs{1};
        for i = 2:numel(sp_subjs)
            common = intersect(common, sp_subjs{i});
        end
        nC = numel(common);
        
        if nC >= 3
            sum_contra_n = zeros(nC, 1); sum_ipsi_n = zeros(nC, 1);
            sum_contra_c = zeros(nC, 1); sum_ipsi_c = zeros(nC, 1);
            
            for i = 1:numel(sp_tables)
                T = sp_tables{i};
                % 공통 피험자 인덱스 매핑
                idx_map = zeros(nC, 1);
                for j = 1:nC
                    idx_map(j) = find(strcmp(T.RowLabel, common(j)));
                end
                
                sum_contra_n = sum_contra_n + ...
                    (T.Mean_nRMSE_Fx_Contra(idx_map) + T.Mean_nRMSE_Fz_Contra(idx_map)) / 2;
                sum_ipsi_n = sum_ipsi_n + ...
                    (T.Mean_nRMSE_Fx_Ipsi(idx_map) + T.Mean_nRMSE_Fz_Ipsi(idx_map)) / 2;
                sum_contra_c = sum_contra_c + ...
                    (T.Mean_Corr_Fx_Contra(idx_map) + T.Mean_Corr_Fz_Contra(idx_map)) / 2;
                sum_ipsi_c = sum_ipsi_c + ...
                    (T.Mean_Corr_Fx_Ipsi(idx_map) + T.Mean_Corr_Fz_Ipsi(idx_map)) / 2;
            end
            
            nF = numel(sp_tables);
            delta_nrmse_speed{r_idx} = (sum_contra_n / nF) - (sum_ipsi_n / nF);
            delta_corr_speed{r_idx}  = (sum_contra_c / nF) - (sum_ipsi_c / nF);
        end
    end
    
    % =================================================================
    % Standard LOSO: ss123
    % =================================================================
    found_loso = [];
    for p = 1:numel(std_patterns)
        found_loso = dir(fullfile(path_loso_dir, std_patterns(p)));
        if ~isempty(found_loso), break; end
    end
    
    if ~isempty(found_loso)
        fpath = fullfile(path_loso_dir, found_loso(1).name);
        S = load(fpath); fn = fieldnames(S); T = S.(fn{1});
        subj_rows = 2:height(T);
        
        n_contra = (T.Mean_nRMSE_Fx_Contra(subj_rows) + T.Mean_nRMSE_Fz_Contra(subj_rows)) / 2;
        n_ipsi   = (T.Mean_nRMSE_Fx_Ipsi(subj_rows)   + T.Mean_nRMSE_Fz_Ipsi(subj_rows))   / 2;
        c_contra = (T.Mean_Corr_Fx_Contra(subj_rows) + T.Mean_Corr_Fz_Contra(subj_rows)) / 2;
        c_ipsi   = (T.Mean_Corr_Fx_Ipsi(subj_rows)   + T.Mean_Corr_Fz_Ipsi(subj_rows))   / 2;
        
        delta_nrmse_std{r_idx} = n_contra - n_ipsi;
        delta_corr_std{r_idx}  = c_contra - c_ipsi;
    end
end

% =========================================================================
% [2] 가설 1: Data Size Effect on Δ (RM-ANOVA across 5 ratios)
% =========================================================================
fprintf('\n\n--- 가설 1: Data Size down → Δ(Contra-Ipsi) up ---\n');

for method = 1:2
    if method == 1
        method_name = 'Speed LOSO';
        D_n = delta_nrmse_speed; D_c = delta_corr_speed;
    else
        method_name = 'Standard LOSO';
        D_n = delta_nrmse_std; D_c = delta_corr_std;
    end
    
    fprintf('\n  [%s]\n', method_name);
    
    all_ok = true;
    for r = 1:numel(ratio_list)
        if isempty(D_n{r}), all_ok = false; break; end
    end
    if ~all_ok
        fprintf('    일부 ratio 데이터 없음 → 건너뜀\n');
        continue;
    end
    
    nS = min(cellfun(@numel, D_n));
    
    for metric = 1:2
        if metric == 1
            metric_name = 'delta nRMSE'; D = D_n;
        else
            metric_name = 'delta Corr'; D = D_c;
        end
        
        k = numel(ratio_list);
        Y = zeros(nS, k);
        for r = 1:k
            Y(:, r) = D{r}(1:nS);
        end
        
        valid = all(isfinite(Y), 2);
        Y = Y(valid, :);
        n = size(Y, 1);
        
        if n < 3
            fprintf('    %s: 유효 피험자 부족\n', metric_name);
            continue;
        end
        
        % RM-ANOVA
        gm = mean(Y(:));
        subj_m = mean(Y, 2);
        cond_m = mean(Y, 1);
        
        SS_cond  = n * sum((cond_m - gm).^2);
        SS_subj  = k * sum((subj_m - gm).^2);
        SS_total = sum((Y(:) - gm).^2);
        SS_error = SS_total - SS_subj - SS_cond;
        
        df_cond  = k - 1;
        df_error = (n - 1) * (k - 1);
        MS_cond  = SS_cond / df_cond;
        MS_error = SS_error / df_error;
        
        if MS_error > 0
            F_stat = MS_cond / MS_error;
            p_val  = 1 - fcdf(F_stat, df_cond, df_error);
        else
            F_stat = NaN; p_val = NaN;
        end
        
        if p_val < 0.001, sig = '***';
        elseif p_val < 0.01, sig = '**';
        elseif p_val < 0.05, sig = '*';
        else, sig = 'n.s.'; end
        
        fprintf('    %s: F(%d,%d)=%.2f, p=%.4f %s\n', ...
            metric_name, df_cond, df_error, F_stat, p_val, sig);
        
        fprintf('      Ratio:');
        for r = 1:k, fprintf('  %d%%', ratio_list(r)); end
        fprintf('\n      Mean: ');
        for r = 1:k, fprintf('  %.4f', cond_m(r)); end
        fprintf('\n      SD:   ');
        for r = 1:k, fprintf('  %.4f', std(Y(:,r),'omitnan')); end
        fprintf('\n');
        
        % 사후검정: 100% vs 나머지
        if p_val < 0.05
            fprintf('      Post-hoc (100%% vs others, Bonferroni x%d):\n', k-1);
            for r = 2:k
                [~, pt] = ttest(Y(:,1), Y(:,r));
                pb = min(pt * (k-1), 1);
                if pb < 0.001, ps = '***';
                elseif pb < 0.01, ps = '**';
                elseif pb < 0.05, ps = '*';
                else, ps = 'n.s.'; end
                
                d = mean(Y(:,1)) - mean(Y(:,r));
                if d > 0, dir_s = '100%>'; else, dir_s = '100%<'; end
                fprintf('        100%% vs %d%%: p=%.4f %s (%s%d%%)\n', ...
                    ratio_list(r), pb, ps, dir_s, ratio_list(r));
            end
        end
    end
end

% =========================================================================
% [3] 가설 2: Speed LOSO vs Standard LOSO (같은 data size)
% =========================================================================
fprintf('\n\n--- 가설 2: Speed LOSO vs Standard LOSO, delta 비교 ---\n');
fprintf('  %-8s  %-12s  %18s  %18s  %10s  %s\n', ...
    'Ratio', 'Metric', 'Speed LOSO', 'Std LOSO', 'p(Bonf)', 'Sig');
fprintf('  %s\n', repmat('-', 1, 85));

n_ratios = numel(ratio_list);

for metric = 1:2
    if metric == 1
        metric_name = 'delta nRMSE';
        D_sp = delta_nrmse_speed; D_st = delta_nrmse_std;
    else
        metric_name = 'delta Corr';
        D_sp = delta_corr_speed; D_st = delta_corr_std;
    end
    
    p_raw_all = NaN(1, n_ratios);
    test_data = cell(n_ratios, 2);  % {v_sp, v_st}
    
    for r = 1:n_ratios
        if isempty(D_sp{r}) || isempty(D_st{r}), continue; end
        nS = min(numel(D_sp{r}), numel(D_st{r}));
        v_sp = D_sp{r}(1:nS);
        v_st = D_st{r}(1:nS);
        valid = isfinite(v_sp) & isfinite(v_st);
        if sum(valid) < 3, continue; end
        test_data{r,1} = v_sp(valid);
        test_data{r,2} = v_st(valid);
        [~, p_raw_all(r)] = ttest(v_sp(valid), v_st(valid));
    end
    
    n_valid_tests = sum(isfinite(p_raw_all));
    p_bonf_all = min(p_raw_all * n_valid_tests, 1);
    
    for r = 1:n_ratios
        if isnan(p_bonf_all(r))
            fprintf('  %-8s  %-12s  %18s  %18s  %10s  %s\n', ...
                sprintf('%d%%', ratio_list(r)), metric_name, '-', '-', '-', '-');
            continue;
        end
        
        v_sp = test_data{r,1}; v_st = test_data{r,2};
        m_sp = mean(v_sp); sd_sp = std(v_sp);
        m_st = mean(v_st); sd_st = std(v_st);
        
        pb = p_bonf_all(r);
        if pb < 0.001, ps = '***';
        elseif pb < 0.01, ps = '**';
        elseif pb < 0.05, ps = '*';
        else, ps = 'n.s.'; end
        
        if m_sp > m_st, dir_s = 'Sp>St'; else, dir_s = 'Sp<St'; end
        
        fprintf('  %-8s  %-12s  %7.4f +/- %.4f  %7.4f +/- %.4f  p=%.4f  %s %s\n', ...
            sprintf('%d%%', ratio_list(r)), metric_name, ...
            m_sp, sd_sp, m_st, sd_st, pb, ps, dir_s);
    end
    
    if metric == 1, fprintf('  %s\n', repmat('-', 1, 85)); end
end

fprintf('\n%% =======================================================\n');
fprintf('%% [End] Statistical Tests\n');
fprintf('%% =======================================================\n');

%% Figure 6) AP GRF 최대 힘 및 충격량에 대한 추정값과 측정값 간의 상관관계  

mat_paths = { ...
    "D:\바탕화면\데이터\Arm_Swing_Paper\Physres_no_physics\Figure3_loso\ss1.mat", ...
    "D:\바탕화면\데이터\Arm_Swing_Paper\Physres_no_physics\Figure3_loso\ss2.mat", ...
    "D:\바탕화면\데이터\Arm_Swing_Paper\Physres_no_physics\Figure3_loso\ss3.mat"};
ss_names = {'1.0 m/s', '1.25 m/s', '1.5 m/s'};
n_ss = numel(ss_names);

% ============================================
%  [사용자 설정] PREFIX: 'Fx' 또는 'Fz'
% ============================================
PREFIX = 'Fx';
if strcmp(PREFIX,'Fx'), dir_str = 'AP'; else, dir_str = 'Vertical'; end

metric_types  = {'Pos_Peak', 'Neg_Peak', 'Pos_Integral', 'Neg_Integral'};
metric_titles = {'Positive Peak', 'Negative Peak', 'Positive Impulse', 'Negative Impulse'};
metric_units  = {'(N/BW)', '(N/BW)', '(Ns/BW)', '(Ns/BW)'};
n_metrics = numel(metric_types);

% ----------------------------------------------------------------
%  Figure 양식 설정
% ----------------------------------------------------------------
SHOW_XLABEL  = false;
SHOW_YLABEL  = false;
SHOW_TITLE   = false;
SHOW_STATS   = false;

% 부호 반전 옵션
% 순서 = {Pos_Peak, Neg_Peak, Pos_Integral, Neg_Integral}
FLIP_SIGN    = [false, true, false, true];

% 배율 설정 (모든 메트릭에 동일 적용)
% 예) 1 → 그대로 / 100 → ×100 / 1000 → ×1000
SCALE_FACTOR = 1;

% tick/limit 설정 (SCALE_FACTOR 적용 후 기준으로 입력)
AUTO_TICKS  = [false, false, false, false];

x_lim_all   = { [0.14  0.32], ...   % Pos_Peak
                [12  36], ...   % Neg_Peak  (flip 후 양수)
                [ 3   7], ...   % Pos_Integral
                [ 3   7] };     % Neg_Integral (flip 후 양수)
y_lim_all   = x_lim_all;

x_ticks_all = { 0.14:0.06:0.32, ...   % Pos_Peak
                12:6:36, ...   % Neg_Peak
                 3:1:7,  ...   % Pos_Integral
                 3:1:7 };      % Neg_Integral
y_ticks_all = x_ticks_all;

fig_size       = [300 300 330 270];
font_size      = 9;
linewidth_axis = 0.8;
linewidth_line = 1.5;
% ----------------------------------------------------------------

close all;

% ============================================
%  .mat 파일 로드
% ============================================
data_cells = cell(n_ss, 1);
for s = 1:n_ss
    loaded = load(mat_paths{s});
    fn = fieldnames(loaded);
    data_cells{s} = loaded.(fn{1});
    fprintf('로드 완료 [%s]: %s\n', ss_names{s}, mat_paths{s});
end

% 공통 피험자 추출
common_subs = fieldnames(data_cells{1});
for s = 2:n_ss
    common_subs = intersect(common_subs, fieldnames(data_cells{s}));
end
subject_list = common_subs(strncmp(common_subs,'S',1));
n_subs = numel(subject_list);
sub_colors = lines(n_subs);
fprintf('\n 분석 대상: %d명\n', n_subs);

% ============================================
%  Figure: 4개 개별 figure
% ============================================
fig_offset_x = [0, 200, 400, 600];
fig_offset_y = [0,   0,   0,   0];

for m = 1:n_metrics
    METRIC_TYPE = metric_types{m};
    NAME_TRUE = sprintf('%s_%s_True_each', PREFIX, METRIC_TYPE);
    NAME_PRED = sprintf('%s_%s_Pred_each', PREFIX, METRIC_TYPE);

    % 부호 반전 계수
    if FLIP_SIGN(m), sign_factor = -1; else, sign_factor = 1; end

    fprintf('\n======================================================\n');
    fprintf(' [%d/4] %s_%s  (sign=%d, scale=%.4g)\n', ...
        m, PREFIX, METRIC_TYPE, sign_factor, SCALE_FACTOR);
    fprintf('------------------------------------------------------\n');
    fprintf(' Subject    | R       | Slope   | Intercept\n');
    fprintf('------------------------------------------------------\n');

    subs_data = struct();
    g_min = inf; g_max = -inf;
    r_list = []; slope_list = [];

    for i = 1:n_subs
        sub_id = subject_list{i};
        mu_t = zeros(1, n_ss);
        mu_p = zeros(1, n_ss);
        ok = true;

        for s = 1:n_ss
            d = data_cells{s}.(sub_id);
            tv_all = []; pv_all = [];
            for k = 1:numel(d)
                if ~isfield(d(k),'metrics'), continue; end
                met = d(k).metrics;
                if isempty(met), continue; end
                if ~isfield(met, NAME_TRUE), continue; end
                tv = met.(NAME_TRUE)(:);
                pv = met.(NAME_PRED)(:);
                if ~isempty(tv) && ~isempty(pv)
                    nm = min(numel(tv), numel(pv));
                    tv_all = [tv_all; tv(1:nm)];
                    pv_all = [pv_all; pv(1:nm)];
                end
            end
            if isempty(tv_all), ok = false; break; end
            % 부호 반전 + 배율 적용
            mu_t(s) = sign_factor * SCALE_FACTOR * mean(tv_all, 'omitnan');
            mu_p(s) = sign_factor * SCALE_FACTOR * mean(pv_all, 'omitnan');
        end

        subs_data(i).id      = sub_id;
        subs_data(i).valid   = ok;
        subs_data(i).t_means = mu_t;
        subs_data(i).p_means = mu_p;

        if ok
            R = corrcoef(mu_t, mu_p);
            r_val = R(1,2);
            if length(unique(mu_t)) > 1
                pf = polyfit(mu_t, mu_p, 1);
                sl = pf(1); ic = pf(2);
                slope_list = [slope_list; sl];
            else
                sl = NaN; ic = NaN;
            end
            subs_data(i).r_val = r_val;
            subs_data(i).slope = sl;
            if ~isnan(r_val), r_list = [r_list; r_val]; end
            fprintf(' %-10s | %.4f | %.4f | %.4f\n', sub_id, r_val, sl, ic);
            g_min = min([g_min, mu_t, mu_p]);
            g_max = max([g_max, mu_t, mu_p]);
        else
            subs_data(i).r_val = NaN;
            subs_data(i).slope = NaN;
        end
    end

    mean_r  = mean(r_list,    'omitnan');
    mean_sl = mean(slope_list,'omitnan');
    std_sl  = std(slope_list, 'omitnan');
    fprintf('------------------------------------------------------\n');
    fprintf(' Mean R: %.4f  |  Slope: %.4f ± %.4f\n', mean_r, mean_sl, std_sl);

    % ── 개별 Figure 생성 ─────────────────────────────────────
    figure('Color', 'w', ...
        'Units',    'points', ...
        'Position', [fig_size(1) + fig_offset_x(m), ...
                     fig_size(2) + fig_offset_y(m), ...
                     fig_size(3), ...
                     fig_size(4)]);

    ax = axes();
    hold(ax, 'on');
    set(ax, ...
        'FontSize',  font_size, ...
        'FontName',  font_name, ...
        'LineWidth', linewidth_axis, ...
        'TickDir',   'out', ...
        'Box',       'off');

    h_leg = []; leg_names = {};

    for i = 1:n_subs
        if ~subs_data(i).valid, continue; end
        xp = subs_data(i).t_means;
        yp = subs_data(i).p_means;
        c  = sub_colors(i,:);

        h = plot(ax, xp, yp, 'o', ...
            'MarkerSize',      4, ...
            'MarkerFaceColor', c, ...
            'MarkerEdgeColor', 'none', ...
            'LineWidth',       linewidth_line);

        if length(unique(xp)) > 1
            pf  = polyfit(xp, yp, 1);
            xln = [min(xp), max(xp)];
            plot(ax, xln, polyval(pf, xln), '-', ...
                'Color',     [c 0.7], ...
                'LineWidth', linewidth_line);
        end

        h_leg(end+1)     = h;
        leg_names{end+1} = subs_data(i).id;
    end

    % 1:1 identity line
    mg = (g_max - g_min) * 0.1;
    if isnan(mg) || mg == 0, mg = 1; end
    lm1 = g_min - mg;
    lm2 = g_max + mg;
    plot(ax, [lm1 lm2], [lm1 lm2], 'k--', 'LineWidth', linewidth_axis);

    axis(ax, 'equal');

    % ── tick / axis limit 적용 ───────────────────────────────
    if AUTO_TICKS(m)
        xlim(ax, [lm1 lm2]);
        ylim(ax, [lm1 lm2]);
        drawnow;
        ctk = get(ax, 'XTick');
        set(ax, 'YTick', ctk);
    else
        xlim(ax, x_lim_all{m});
        ylim(ax, y_lim_all{m});
        set(ax, 'XTick', x_ticks_all{m});
        set(ax, 'YTick', y_ticks_all{m});
    end
    % ─────────────────────────────────────────────────────────

    % xlabel / ylabel 조건부 표시
    if SHOW_XLABEL
        xlabel(ax, sprintf('Measured %s', metric_units{m}), ...
            'FontSize', font_size, 'FontName', font_name);
    end
    if SHOW_YLABEL
        ylabel(ax, sprintf('Estimated %s', metric_units{m}), ...
            'FontSize', font_size, 'FontName', font_name);
    end

    % title / stats 조건부 표시
    if SHOW_TITLE || SHOW_STATS
        if SHOW_TITLE && SHOW_STATS
            title_str = sprintf('%s %s\nR=%.3f, Slope=%.2f±%.2f', ...
                dir_str, metric_titles{m}, mean_r, mean_sl, std_sl);
        elseif SHOW_TITLE && ~SHOW_STATS
            title_str = sprintf('%s %s', dir_str, metric_titles{m});
        else
            title_str = sprintf('R=%.3f, Slope=%.2f±%.2f', mean_r, mean_sl, std_sl);
        end
        title(ax, title_str, ...
            'FontWeight', 'normal', ...
            'FontSize',   font_size, ...
            'FontName',   font_name);
    end

    hold(ax, 'off');
end

% Stride-level R and Slope for AP GRF clinical metrics
% 기존 코드와 동일한 데이터 로드 구조 사용

mat_paths = { ...
    "D:\바탕화면\데이터\Arm_Swing_Paper\Physres_no_physics\Figure3_loso\ss1.mat", ...
    "D:\바탕화면\데이터\Arm_Swing_Paper\Physres_no_physics\Figure3_loso\ss2.mat", ...
    "D:\바탕화면\데이터\Arm_Swing_Paper\Physres_no_physics\Figure3_loso\ss3.mat"};
ss_names = {'1.0 m/s', '1.25 m/s', '1.5 m/s'};
n_ss = numel(ss_names);

PREFIX = 'Fx';
metric_types  = {'Pos_Peak', 'Neg_Peak', 'Pos_Integral', 'Neg_Integral'};
metric_titles = {'Positive Peak (Propulsion)', 'Negative Peak (Braking)', ...
                 'Positive Impulse', 'Negative Impulse'};
FLIP_SIGN = [false, true, false, true];
SCALE_FACTOR = 1;

% ============================================
%  .mat 파일 로드
% ============================================
data_cells = cell(n_ss, 1);
for s = 1:n_ss
    loaded = load(mat_paths{s});
    fn = fieldnames(loaded);
    data_cells{s} = loaded.(fn{1});
    fprintf('로드 완료 [%s]: %s\n', ss_names{s}, mat_paths{s});
end

% 공통 피험자 추출
common_subs = fieldnames(data_cells{1});
for s = 2:n_ss
    common_subs = intersect(common_subs, fieldnames(data_cells{s}));
end
subject_list = common_subs(strncmp(common_subs,'S',1));
n_subs = numel(subject_list);

fprintf('\n 분석 대상: %d명\n\n', n_subs);

% ============================================
%  Stride-level 계산
% ============================================
fprintf('=======================================================\n');
fprintf(' [STRIDE-LEVEL] AP GRF Clinical Metrics\n');
fprintf('=======================================================\n');

for m = 1:numel(metric_types)
    METRIC_TYPE = metric_types{m};
    NAME_TRUE = sprintf('%s_%s_True_each', PREFIX, METRIC_TYPE);
    NAME_PRED = sprintf('%s_%s_Pred_each', PREFIX, METRIC_TYPE);
    
    if FLIP_SIGN(m), sign_factor = -1; else, sign_factor = 1; end
    
    % 전체 stride 값 수집
    all_true = [];
    all_pred = [];
    
    % 피험자별 stride 값 수집 (subject-level R도 함께)
    sub_r_list = [];
    sub_slope_list = [];
    
    fprintf('\n [%d/4] %s\n', m, metric_titles{m});
    fprintf('------------------------------------------------------\n');
    fprintf(' Subject    | stride R | stride Slope\n');
    fprintf('------------------------------------------------------\n');
    
    for i = 1:n_subs
        sub_id = subject_list{i};
        sub_true = [];
        sub_pred = [];
        
        for s = 1:n_ss
            d = data_cells{s}.(sub_id);
            for k = 1:numel(d)
                if ~isfield(d(k),'metrics'), continue; end
                met = d(k).metrics;
                if isempty(met), continue; end
                if ~isfield(met, NAME_TRUE), continue; end
                tv = met.(NAME_TRUE)(:);
                pv = met.(NAME_PRED)(:);
                if ~isempty(tv) && ~isempty(pv)
                    nm = min(numel(tv), numel(pv));
                    tv = sign_factor * SCALE_FACTOR * tv(1:nm);
                    pv = sign_factor * SCALE_FACTOR * pv(1:nm);
                    sub_true = [sub_true; tv];
                    sub_pred = [sub_pred; pv];
                end
            end
        end
        
        if numel(sub_true) > 1
            % 피험자별 stride-level R, slope
            R_mat = corrcoef(sub_true, sub_pred);
            sub_r = R_mat(1,2);
            pf = polyfit(sub_true, sub_pred, 1);
            sub_sl = pf(1);
            
            sub_r_list = [sub_r_list; sub_r];
            sub_slope_list = [sub_slope_list; sub_sl];
            
            fprintf(' %-10s | %.4f   | %.4f\n', sub_id, sub_r, sub_sl);
            
            all_true = [all_true; sub_true];
            all_pred = [all_pred; sub_pred];
        end
    end
    
    % 전체 stride pooled R, slope
    if numel(all_true) > 1
        R_all = corrcoef(all_true, all_pred);
        r_pooled = R_all(1,2);
        pf_all = polyfit(all_true, all_pred, 1);
        slope_pooled = pf_all(1);
        intercept_pooled = pf_all(2);
    end
    
    fprintf('------------------------------------------------------\n');
    fprintf(' [Subject-mean] R: %.4f ± %.4f  |  Slope: %.4f ± %.4f\n', ...
        mean(sub_r_list,'omitnan'), std(sub_r_list,'omitnan'), ...
        mean(sub_slope_list,'omitnan'), std(sub_slope_list,'omitnan'));
    fprintf(' [Pooled-all]   R: %.4f          |  Slope: %.4f  (intercept: %.4f)\n', ...
        r_pooled, slope_pooled, intercept_pooled);
    fprintf(' Total strides: %d\n', numel(all_true));
    fprintf('=======================================================\n');
end



%% Figure 9) Speed LOSO: Ipsilateral vs Contralateral
% (a) Correlation  (b) nRMSE
% 점 두 개 (Ipsi, Contra) + 사이 영역 채우기
clc; close all;

% =========================================================================
% 0. 표시 옵션 (ON/OFF)
% =========================================================================
SHOW_XLABEL      = false;
SHOW_YLABEL      = false;
SHOW_LEGEND      = false;
SHOW_ERRORBAR    = true;

% =========================================================================
% 1. 축 설정 (수동 / 자동)  [] = 자동
% =========================================================================
% --- Correlation ---
corr_x_lim_val  = [];
corr_x_ticks    = [];
corr_x_tick_lbl = {5 10 20 50 100};
corr_y_lim_val  = [0.84, 1.00];              
corr_y_ticks    = 0.84:0.04:1.00;              

% --- nRMSE ---
nrmse_x_lim_val  = [];
nrmse_x_ticks    = [];
nrmse_x_tick_lbl = {5 10 20 50 100};
nrmse_y_lim_val  = [0.08 0.16];
nrmse_y_ticks    = 0.08:0.02:0.16;

% =========================================================================
% 2. 공통 스타일 설정
% =========================================================================
fig_size       = [150 150 180 160];
font_size      = 8;
linewidth_axis        = 0.8;
linewidth_line        = 1.5;
face_alpha     = 0.25;
marker_size    = 4.5;

% =========================================================================
% 3. 색상 정의
% =========================================================================
col_ipsi   = [0 80 158] / 255;
col_contra = [192 80 77] / 255;
col_fill   = (col_ipsi + col_contra) / 2;

% =========================================================================
% 4. 데이터 설정
% =========================================================================
base_root    = "D:\바탕화면\데이터\Arm_Swing_Paper";
target_model = "Physres_no_physics";
ratio_list   = [5, 10, 20, 50, 100];
labels       = {'5%', '10%', '20%', '50%', '100%'};

speed_patterns = ["*train_ss1*.mat", "*train_ss2*.mat", "*train_ss3*.mat"];

n_ratios = numel(ratio_list);

% nRMSE
ipsi_nrmse_mu   = NaN(n_ratios, 1);  ipsi_nrmse_sd   = NaN(n_ratios, 1);
contra_nrmse_mu = NaN(n_ratios, 1);  contra_nrmse_sd = NaN(n_ratios, 1);

% Correlation
ipsi_corr_mu   = NaN(n_ratios, 1);  ipsi_corr_sd   = NaN(n_ratios, 1);
contra_corr_mu = NaN(n_ratios, 1);  contra_corr_sd = NaN(n_ratios, 1);

% =========================================================================
% 5. 각 ratio별 Speed LOSO에서 Ipsi / Contra 계산
% =========================================================================
for r_idx = 1:n_ratios
    curr_ratio = ratio_list(r_idx);
    
    if curr_ratio == 100
        path_speed_dir = fullfile(base_root, target_model, "Figure4_speedloso");
    else
        sub_folder = sprintf("%dpercent", curr_ratio);
        path_speed_dir = fullfile(base_root, target_model, "Figure6_datasize_sploso", sub_folder);
    end
    
    sp_tables = {};
    sp_subjs  = {};
    for i = 1:3
        found = dir(fullfile(path_speed_dir, speed_patterns(i)));
        if isempty(found), continue; end
        fpath = fullfile(path_speed_dir, found(1).name);
        S = load(fpath); fn = fieldnames(S); T = S.(fn{1});
        sp_tables{end+1} = T;
        sp_subjs{end+1}  = T.RowLabel(2:end);
    end
    
    if numel(sp_tables) >= 1
        common = sp_subjs{1};
        for i = 2:numel(sp_subjs)
            common = intersect(common, sp_subjs{i});
        end
        nC = numel(common);
        
        if nC >= 1
            sum_nrmse_ipsi   = zeros(nC, 1);
            sum_nrmse_contra = zeros(nC, 1);
            sum_corr_ipsi    = zeros(nC, 1);
            sum_corr_contra  = zeros(nC, 1);
            
            for i = 1:numel(sp_tables)
                T = sp_tables{i};
                idx_map = zeros(nC, 1);
                for j = 1:nC
                    idx_map(j) = find(strcmp(T.RowLabel, common(j)));
                end
                
                % nRMSE
                sum_nrmse_ipsi = sum_nrmse_ipsi + ...
                    (T.Mean_nRMSE_Fx_Ipsi(idx_map) + T.Mean_nRMSE_Fz_Ipsi(idx_map)) / 2;
                sum_nrmse_contra = sum_nrmse_contra + ...
                    (T.Mean_nRMSE_Fx_Contra(idx_map) + T.Mean_nRMSE_Fz_Contra(idx_map)) / 2;
                
                % Correlation
                sum_corr_ipsi = sum_corr_ipsi + ...
                    (T.Mean_Corr_Fx_Ipsi(idx_map) + T.Mean_Corr_Fz_Ipsi(idx_map)) / 2;
                sum_corr_contra = sum_corr_contra + ...
                    (T.Mean_Corr_Fx_Contra(idx_map) + T.Mean_Corr_Fz_Contra(idx_map)) / 2;
            end
            
            nF = numel(sp_tables);
            
            % nRMSE (×100 for %)
            avg_nrmse_ipsi   = sum_nrmse_ipsi / nF ;
            avg_nrmse_contra = sum_nrmse_contra / nF ;
            ipsi_nrmse_mu(r_idx)   = mean(avg_nrmse_ipsi, 'omitnan');
            ipsi_nrmse_sd(r_idx)   = std(avg_nrmse_ipsi, 0, 'omitnan');
            contra_nrmse_mu(r_idx) = mean(avg_nrmse_contra, 'omitnan');
            contra_nrmse_sd(r_idx) = std(avg_nrmse_contra, 0, 'omitnan');
            
            % Correlation
            avg_corr_ipsi   = sum_corr_ipsi / nF;
            avg_corr_contra = sum_corr_contra / nF;
            ipsi_corr_mu(r_idx)   = mean(avg_corr_ipsi, 'omitnan');
            ipsi_corr_sd(r_idx)   = std(avg_corr_ipsi, 0, 'omitnan');
            contra_corr_mu(r_idx) = mean(avg_corr_contra, 'omitnan');
            contra_corr_sd(r_idx) = std(avg_corr_contra, 0, 'omitnan');
        end
    end
end

% 결과 출력
fprintf('\n=== Speed LOSO: Ipsi vs Contra ===\n');
fprintf('\n--- Correlation ---\n');
fprintf('%-8s  %18s  %18s\n', 'Ratio', 'Ipsilateral', 'Contralateral');
for r = 1:n_ratios
    fprintf('%-8s  %7.3f +/- %5.3f  %7.3f +/- %5.3f\n', labels{r}, ...
        ipsi_corr_mu(r), ipsi_corr_sd(r), contra_corr_mu(r), contra_corr_sd(r));
end
fprintf('\n--- nRMSE (%%) ---\n');
fprintf('%-8s  %18s  %18s\n', 'Ratio', 'Ipsilateral', 'Contralateral');
for r = 1:n_ratios
    fprintf('%-8s  %7.2f +/- %5.2f  %7.2f +/- %5.2f\n', labels{r}, ...
        ipsi_nrmse_mu(r), ipsi_nrmse_sd(r), contra_nrmse_mu(r), contra_nrmse_sd(r));
end

% =========================================================================
% 6. 축 자동 계산 (빈 값일 경우)
% =========================================================================
x = 1:n_ratios;

% --- Correlation 축 ---
if isempty(corr_x_lim_val),  corr_x_lim_val  = [0.5, n_ratios + 0.5]; end
if isempty(corr_x_ticks),    corr_x_ticks    = x;                      end
if isempty(corr_x_tick_lbl), corr_x_tick_lbl = labels;                  end
if isempty(corr_y_lim_val) || isempty(corr_y_ticks)
    all_c = [ipsi_corr_mu + ipsi_corr_sd; ipsi_corr_mu - ipsi_corr_sd; ...
             contra_corr_mu + contra_corr_sd; contra_corr_mu - contra_corr_sd];
    c_margin = (max(all_c) - min(all_c)) * 0.2;
    if isempty(corr_y_lim_val)
        corr_y_lim_val = [min(all_c) - c_margin, max(all_c) + c_margin];
    end
    if isempty(corr_y_ticks)
        corr_y_ticks = linspace(corr_y_lim_val(1), corr_y_lim_val(2), 5);
        corr_y_ticks = round(corr_y_ticks, 3);
    end
end

% --- nRMSE 축 ---
if isempty(nrmse_x_lim_val),  nrmse_x_lim_val  = [0.5, n_ratios + 0.5]; end
if isempty(nrmse_x_ticks),    nrmse_x_ticks    = x;                      end
if isempty(nrmse_x_tick_lbl), nrmse_x_tick_lbl = labels;                  end
if isempty(nrmse_y_lim_val) || isempty(nrmse_y_ticks)
    all_n = [ipsi_nrmse_mu + ipsi_nrmse_sd; ipsi_nrmse_mu - ipsi_nrmse_sd; ...
             contra_nrmse_mu + contra_nrmse_sd; contra_nrmse_mu - contra_nrmse_sd];
    n_margin = (max(all_n) - min(all_n)) * 0.2;
    if isempty(nrmse_y_lim_val)
        nrmse_y_lim_val = [min(all_n) - n_margin, max(all_n) + n_margin];
    end
    if isempty(nrmse_y_ticks)
        nrmse_y_ticks = linspace(nrmse_y_lim_val(1), nrmse_y_lim_val(2), 5);
        nrmse_y_ticks = round(nrmse_y_ticks, 1);
    end
end

% =========================================================================
% 7. 공통 그리기 함수
% =========================================================================
function draw_ipsi_contra(x, mu_ipsi, sd_ipsi, mu_contra, sd_contra, ...
    col_ipsi, col_contra, col_fill, ...
    xl, yl, xtk, xtkl, ytk, ...
    fig_size, font_size, lw_axis, lw_line, face_alpha, mk_size, ...
    SHOW_ERRORBAR, SHOW_XLABEL, SHOW_YLABEL, SHOW_LEGEND, ...
    xlabel_str, ylabel_str, fig_name)

    figure('Name', fig_name, 'Position', fig_size, 'Color', 'w');
    hold on;
    
    % --- 두 라인 사이 영역 채우기 ---
    fill_x = [x, fliplr(x)];
    fill_y = [mu_ipsi', fliplr(mu_contra')];
    fill(fill_x, fill_y, col_fill, ...
        'FaceAlpha', face_alpha, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    % --- Ipsilateral ---
    if SHOW_ERRORBAR
        errorbar(x, mu_ipsi, sd_ipsi, ...
            'Color', col_ipsi * 0.7, 'LineWidth', lw_axis, ...
            'LineStyle', 'none', 'CapSize', 4, 'HandleVisibility', 'off');
    end
    plot(x, mu_ipsi, '-', 'Color', col_ipsi * 0.7, 'LineWidth', lw_line);
    plot(x, mu_ipsi, 'o', ...
        'MarkerSize', mk_size, 'MarkerFaceColor', col_ipsi, ...
        'MarkerEdgeColor', col_ipsi * 0.7, 'LineWidth', lw_axis, ...
        'HandleVisibility', 'off');
    
    % --- Contralateral ---
    if SHOW_ERRORBAR
        errorbar(x, mu_contra, sd_contra, ...
            'Color', col_contra * 0.7, 'LineWidth', lw_axis, ...
            'LineStyle', 'none', 'CapSize', 4, 'HandleVisibility', 'off');
    end
    plot(x, mu_contra, '-', 'Color', col_contra * 0.7, 'LineWidth', lw_line);
    plot(x, mu_contra, 'o', ...
        'MarkerSize', mk_size, 'MarkerFaceColor', col_contra, ...
        'MarkerEdgeColor', col_contra * 0.7, 'LineWidth', lw_axis, ...
        'HandleVisibility', 'off');
    
    % --- 스타일 ---
    set(gca, ...
        'FontName', 'Helvetica', 'FontSize', font_size, ...
        'Box', 'off', 'TickDir', 'out', 'LineWidth', lw_axis, ...
        'XLim', xl, 'YLim', yl, ...
        'XTick', xtk, 'XTickLabel', xtkl, ...
        'YTick', ytk, ...
        'XGrid', 'off', 'YGrid', 'off', ...
        'XColor', 'k', 'YColor', 'k');
    
    if SHOW_XLABEL, xlabel(xlabel_str, 'FontName', 'Helvetica', 'FontSize', font_size); end
    if SHOW_YLABEL, ylabel(ylabel_str, 'FontName', 'Helvetica', 'FontSize', font_size); end
    
    if SHOW_LEGEND
        legend({'Ipsilateral', 'Contralateral'}, ...
            'Location', 'best', 'Box', 'off', ...
            'FontName', 'Helvetica', 'FontSize', font_size);
    end
    
    hold off;
end

% =========================================================================
% 8. (a) Correlation 그래프
% =========================================================================
draw_ipsi_contra(x, ipsi_corr_mu, ipsi_corr_sd, contra_corr_mu, contra_corr_sd, ...
    col_ipsi, col_contra, col_fill, ...
    corr_x_lim_val, corr_y_lim_val, corr_x_ticks, corr_x_tick_lbl, corr_y_ticks, ...
    fig_size, font_size, linewidth_axis, linewidth_line, face_alpha, marker_size, ...
    SHOW_ERRORBAR, SHOW_XLABEL, SHOW_YLABEL, SHOW_LEGEND, ...
    'Dataset Size', 'Correlation (R)', 'Correlation: Ipsi vs Contra');

% =========================================================================
% 9. (b) nRMSE 그래프
% =========================================================================
draw_ipsi_contra(x, ipsi_nrmse_mu, ipsi_nrmse_sd, contra_nrmse_mu, contra_nrmse_sd, ...
    col_ipsi, col_contra, col_fill, ...
    nrmse_x_lim_val, nrmse_y_lim_val, nrmse_x_ticks, nrmse_x_tick_lbl, nrmse_y_ticks, ...
    fig_size, font_size, linewidth_axis, linewidth_line, face_alpha, marker_size, ...
    SHOW_ERRORBAR, SHOW_XLABEL, SHOW_YLABEL, SHOW_LEGEND, ...
    'Dataset Size', 'nRMSE (%)', 'nRMSE: Ipsi vs Contra');

%% Fig 8-1) Ipsi vs Contra 통계 파트
%  통계 검정: 데이터 크기 감소 → Ipsi-Contra 격차 증가
%  검정 방법:
%    1. 각 ratio에서 Ipsi vs Contra paired t-test
%    2. 격차(Ipsi-Contra)에 대한 RM-ANOVA (ratio 효과)
%    3. Linear trend test (격차 vs ratio)
%    4. Post-hoc 비교 (격차 간)
% ========================================================================
clc;

% =========================================================================
% 1. 피험자별 Ipsi-Contra 격차 계산 (기존 코드에서 데이터 수집)
% =========================================================================
base_root    = "D:\바탕화면\데이터\Arm_Swing_Paper";
target_model = "Physres_no_physics";
ratio_list   = [5, 10, 20, 50, 100];
labels       = {'5%', '10%', '20%', '50%', '100%'};
speed_patterns = ["*train_ss1*.mat", "*train_ss2*.mat", "*train_ss3*.mat"];
n_ratios = numel(ratio_list);

% 피험자별 데이터 저장 (nRMSE, Correlation)
subj_diff_nrmse = [];  % [n_subj x n_ratios]
subj_diff_corr  = [];  % [n_subj x n_ratios]
subj_ipsi_nrmse = [];
subj_contra_nrmse = [];
subj_ipsi_corr = [];
subj_contra_corr = [];

common_subj_all = {};

for r_idx = 1:n_ratios
    curr_ratio = ratio_list(r_idx);
    
    if curr_ratio == 100
        path_speed_dir = fullfile(base_root, target_model, "Figure4_speedloso");
    else
        sub_folder = sprintf("%dpercent", curr_ratio);
        path_speed_dir = fullfile(base_root, target_model, "Figure6_datasize_sploso", sub_folder);
    end
    
    sp_tables = {};
    sp_subjs  = {};
    for i = 1:3
        found = dir(fullfile(path_speed_dir, speed_patterns(i)));
        if isempty(found), continue; end
        fpath = fullfile(path_speed_dir, found(1).name);
        S = load(fpath); fn = fieldnames(S); T = S.(fn{1});
        sp_tables{end+1} = T;
        sp_subjs{end+1}  = T.RowLabel(2:end);
    end
    
    if numel(sp_tables) >= 1
        common = sp_subjs{1};
        for i = 2:numel(sp_subjs)
            common = intersect(common, sp_subjs{i});
        end
        nC = numel(common);
        
        if r_idx == 1
            common_subj_all = common;
        else
            common_subj_all = intersect(common_subj_all, common);
        end
    end
end

% 공통 피험자로 다시 계산
nSubj = numel(common_subj_all);
subj_ipsi_nrmse   = NaN(nSubj, n_ratios);
subj_contra_nrmse = NaN(nSubj, n_ratios);
subj_ipsi_corr    = NaN(nSubj, n_ratios);
subj_contra_corr  = NaN(nSubj, n_ratios);

for r_idx = 1:n_ratios
    curr_ratio = ratio_list(r_idx);
    
    if curr_ratio == 100
        path_speed_dir = fullfile(base_root, target_model, "Figure4_speedloso");
    else
        sub_folder = sprintf("%dpercent", curr_ratio);
        path_speed_dir = fullfile(base_root, target_model, "Figure6_datasize_sploso", sub_folder);
    end
    
    sp_tables = {};
    for i = 1:3
        found = dir(fullfile(path_speed_dir, speed_patterns(i)));
        if isempty(found), continue; end
        fpath = fullfile(path_speed_dir, found(1).name);
        S = load(fpath); fn = fieldnames(S); T = S.(fn{1});
        sp_tables{end+1} = T;
    end
    
    nF = numel(sp_tables);
    
    for s_idx = 1:nSubj
        subj_name = common_subj_all{s_idx};
        
        sum_ipsi_nrmse = 0;
        sum_contra_nrmse = 0;
        sum_ipsi_corr = 0;
        sum_contra_corr = 0;
        
        for i = 1:nF
            T = sp_tables{i};
            row_idx = find(strcmp(T.RowLabel, subj_name));
            if isempty(row_idx), continue; end
            
            sum_ipsi_nrmse = sum_ipsi_nrmse + ...
                (T.Mean_nRMSE_Fx_Ipsi(row_idx) + T.Mean_nRMSE_Fz_Ipsi(row_idx)) / 2;
            sum_contra_nrmse = sum_contra_nrmse + ...
                (T.Mean_nRMSE_Fx_Contra(row_idx) + T.Mean_nRMSE_Fz_Contra(row_idx)) / 2;
            sum_ipsi_corr = sum_ipsi_corr + ...
                (T.Mean_Corr_Fx_Ipsi(row_idx) + T.Mean_Corr_Fz_Ipsi(row_idx)) / 2;
            sum_contra_corr = sum_contra_corr + ...
                (T.Mean_Corr_Fx_Contra(row_idx) + T.Mean_Corr_Fz_Contra(row_idx)) / 2;
        end
        
        subj_ipsi_nrmse(s_idx, r_idx)   = sum_ipsi_nrmse / nF;
        subj_contra_nrmse(s_idx, r_idx) = sum_contra_nrmse / nF;
        subj_ipsi_corr(s_idx, r_idx)    = sum_ipsi_corr / nF;
        subj_contra_corr(s_idx, r_idx)  = sum_contra_corr / nF;
    end
end

% 격차 계산: Contra - Ipsi (Contra가 더 나쁘다고 가정)
diff_nrmse = subj_contra_nrmse - subj_ipsi_nrmse;  % 양수 = Contra가 더 높은 오차
diff_corr  = subj_ipsi_corr - subj_contra_corr;     % 양수 = Ipsi가 더 높은 상관

fprintf('========================================\n');
fprintf('  공통 피험자 수: %d명\n', nSubj);
fprintf('========================================\n\n');

% =========================================================================
% 2. 각 Ratio에서 Ipsi vs Contra Paired t-test
% =========================================================================
fprintf('========================================\n');
fprintf('  Test 1: Paired t-test (Ipsi vs Contra at each ratio)\n');
fprintf('========================================\n');

fprintf('\n--- nRMSE ---\n');
fprintf('%-8s  %10s  %10s  %12s  %10s  %8s\n', 'Ratio', 'Ipsi', 'Contra', 't-stat', 'p-value', 'Sig');
for r = 1:n_ratios
    ipsi_vals = subj_ipsi_nrmse(:, r);
    contra_vals = subj_contra_nrmse(:, r);
    
    [~, p, ~, stats] = ttest(contra_vals, ipsi_vals);
    
    if p < 0.001, sig = '***';
    elseif p < 0.01, sig = '**';
    elseif p < 0.05, sig = '*';
    else, sig = 'n.s.'; end
    
    fprintf('%-8s  %10.4f  %10.4f  %12.3f  %10.4f  %8s\n', ...
        labels{r}, mean(ipsi_vals), mean(contra_vals), stats.tstat, p, sig);
end

fprintf('\n--- Correlation ---\n');
fprintf('%-8s  %10s  %10s  %12s  %10s  %8s\n', 'Ratio', 'Ipsi', 'Contra', 't-stat', 'p-value', 'Sig');
for r = 1:n_ratios
    ipsi_vals = subj_ipsi_corr(:, r);
    contra_vals = subj_contra_corr(:, r);
    
    [~, p, ~, stats] = ttest(ipsi_vals, contra_vals);
    
    if p < 0.001, sig = '***';
    elseif p < 0.01, sig = '**';
    elseif p < 0.05, sig = '*';
    else, sig = 'n.s.'; end
    
    fprintf('%-8s  %10.4f  %10.4f  %12.3f  %10.4f  %8s\n', ...
        labels{r}, mean(ipsi_vals), mean(contra_vals), stats.tstat, p, sig);
end

% =========================================================================
% 3. RM-ANOVA: 격차(Diff)에 대한 Ratio 효과
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Test 2: RM-ANOVA on Gap (Contra - Ipsi)\n');
fprintf('  H0: 격차가 ratio에 따라 변하지 않는다\n');
fprintf('========================================\n');

run_rmanova_gap = @(Y, name) run_rmanova_with_posthoc(Y, name, labels);

fprintf('\n--- nRMSE Gap (Contra - Ipsi) ---\n');
run_rmanova_gap(diff_nrmse, 'nRMSE Gap');

fprintf('\n--- Correlation Gap (Ipsi - Contra) ---\n');
run_rmanova_gap(diff_corr, 'Correlation Gap');

% =========================================================================
% 4. Linear Trend Test: 격차 vs Ratio
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Test 3: Linear Trend (Gap vs Dataset Size)\n');
fprintf('  H0: 격차와 데이터 크기 간 선형 관계 없음\n');
fprintf('========================================\n');

% Log-transformed ratio for better linearity
log_ratio = log10(ratio_list);  % [5, 10, 20, 50, 100] → log scale

fprintf('\n--- nRMSE Gap ---\n');
mean_diff_nrmse = mean(diff_nrmse, 1);
[r_nrmse, p_nrmse] = corr(log_ratio', mean_diff_nrmse', 'Type', 'Pearson');
fprintf('Pearson r (log_ratio vs gap): r = %.4f, p = %.4f\n', r_nrmse, p_nrmse);

% 개별 피험자 데이터로 mixed-effects style 분석
all_log_ratio = repmat(log_ratio, nSubj, 1);
all_diff_nrmse = diff_nrmse(:);
all_log_ratio = all_log_ratio(:);

mdl_nrmse = fitlm(all_log_ratio, all_diff_nrmse);
fprintf('Linear regression: β = %.4f, t = %.3f, p = %.4f\n', ...
    mdl_nrmse.Coefficients.Estimate(2), ...
    mdl_nrmse.Coefficients.tStat(2), ...
    mdl_nrmse.Coefficients.pValue(2));
fprintf('  → β < 0 이면: 데이터 줄수록 격차 증가\n');

fprintf('\n--- Correlation Gap ---\n');
mean_diff_corr = mean(diff_corr, 1);
[r_corr, p_corr] = corr(log_ratio', mean_diff_corr', 'Type', 'Pearson');
fprintf('Pearson r (log_ratio vs gap): r = %.4f, p = %.4f\n', r_corr, p_corr);

all_diff_corr = diff_corr(:);
mdl_corr = fitlm(all_log_ratio, all_diff_corr);
fprintf('Linear regression: β = %.4f, t = %.3f, p = %.4f\n', ...
    mdl_corr.Coefficients.Estimate(2), ...
    mdl_corr.Coefficients.tStat(2), ...
    mdl_corr.Coefficients.pValue(2));
fprintf('  → β < 0 이면: 데이터 줄수록 격차 증가\n');

% =========================================================================
% 5. Post-hoc: 5% vs 100% 격차 비교 (Paired t-test)
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Test 4: Post-hoc - 5%% vs 100%% Gap Comparison\n');
fprintf('========================================\n');

% nRMSE gap: 5% vs 100%
gap_5_nrmse = diff_nrmse(:, 1);   % 5%
gap_100_nrmse = diff_nrmse(:, 5); % 100%
[~, p_posthoc_nrmse, ~, stats_nrmse] = ttest(gap_5_nrmse, gap_100_nrmse);
cohens_d_nrmse = mean(gap_5_nrmse - gap_100_nrmse) / std(gap_5_nrmse - gap_100_nrmse);

fprintf('\n--- nRMSE Gap ---\n');
fprintf('Gap at 5%%:   %.4f ± %.4f\n', mean(gap_5_nrmse), std(gap_5_nrmse));
fprintf('Gap at 100%%: %.4f ± %.4f\n', mean(gap_100_nrmse), std(gap_100_nrmse));
fprintf('Paired t-test: t(%d) = %.3f, p = %.4f, Cohen''s d = %.3f\n', ...
    stats_nrmse.df, stats_nrmse.tstat, p_posthoc_nrmse, cohens_d_nrmse);

% Correlation gap: 5% vs 100%
gap_5_corr = diff_corr(:, 1);
gap_100_corr = diff_corr(:, 5);
[~, p_posthoc_corr, ~, stats_corr] = ttest(gap_5_corr, gap_100_corr);
cohens_d_corr = mean(gap_5_corr - gap_100_corr) / std(gap_5_corr - gap_100_corr);

fprintf('\n--- Correlation Gap ---\n');
fprintf('Gap at 5%%:   %.4f ± %.4f\n', mean(gap_5_corr), std(gap_5_corr));
fprintf('Gap at 100%%: %.4f ± %.4f\n', mean(gap_100_corr), std(gap_100_corr));
fprintf('Paired t-test: t(%d) = %.3f, p = %.4f, Cohen''s d = %.3f\n', ...
    stats_corr.df, stats_corr.tstat, p_posthoc_corr, cohens_d_corr);

% =========================================================================
% 6. 요약 테이블 출력
% =========================================================================
fprintf('\n========================================\n');
fprintf('  Summary: Gap by Dataset Size\n');
fprintf('========================================\n');
fprintf('\n%-8s  %16s  %16s\n', 'Ratio', 'nRMSE Gap', 'Corr Gap');
fprintf('%-8s  %16s  %16s\n', '', '(Contra-Ipsi)', '(Ipsi-Contra)');
fprintf('%s\n', repmat('-', 1, 45));
for r = 1:n_ratios
    fprintf('%-8s  %6.4f ± %6.4f  %6.4f ± %6.4f\n', ...
        labels{r}, ...
        mean(diff_nrmse(:,r)), std(diff_nrmse(:,r)), ...
        mean(diff_corr(:,r)), std(diff_corr(:,r)));
end

fprintf('\n=== 해석 ===\n');
fprintf('nRMSE Gap > 0: Contralateral 오차가 더 큼\n');
fprintf('Corr Gap > 0: Ipsilateral 상관이 더 높음\n');
fprintf('β < 0 (Linear Trend): 데이터 크기 감소 → 격차 증가\n');

% =========================================================================
% RM-ANOVA 함수
% =========================================================================
function run_rmanova_with_posthoc(Y, var_name, ratio_labels)
    % Y: [n_subj x n_ratios]
    valid = all(isfinite(Y), 2);
    Y = Y(valid, :);
    n = size(Y, 1);
    k = size(Y, 2);
    
    if n < 3
        fprintf('  유효 피험자 부족 (n=%d)\n', n);
        return;
    end
    
    % RM-ANOVA 계산
    gm = mean(Y(:));
    subj_m = mean(Y, 2);
    cond_m = mean(Y, 1);
    
    SS_cond  = n * sum((cond_m - gm).^2);
    SS_subj  = k * sum((subj_m - gm).^2);
    SS_total = sum((Y(:) - gm).^2);
    SS_error = SS_total - SS_subj - SS_cond;
    
    df_cond  = k - 1;
    df_error = (n - 1) * (k - 1);
    MS_cond  = SS_cond / df_cond;
    MS_error = SS_error / df_error;
    
    if MS_error > 0
        F_stat = MS_cond / MS_error;
        p_val  = 1 - fcdf(F_stat, df_cond, df_error);
    else
        F_stat = NaN; p_val = NaN;
    end
    
    eta_sq = SS_cond / (SS_cond + SS_error);
    
    if p_val < 0.001, sig = '***';
    elseif p_val < 0.01, sig = '**';
    elseif p_val < 0.05, sig = '*';
    else, sig = 'n.s.'; end
    
    fprintf('RM-ANOVA: F(%d,%d) = %.2f, p = %.4f %s, η²p = %.3f\n', ...
        df_cond, df_error, F_stat, p_val, sig, eta_sq);
    
    % Post-hoc (유의할 경우)
    if p_val < 0.05
        fprintf('Post-hoc (Bonferroni corrected):\n');
        
        % 5% vs others
        comparisons = {[1,2], [1,3], [1,4], [1,5]};  % 5% vs 10%, 20%, 50%, 100%
        comp_names = {'5% vs 10%', '5% vs 20%', '5% vs 50%', '5% vs 100%'};
        n_comp = numel(comparisons);
        
        for iC = 1:n_comp
            c = comparisons{iC};
            x1 = Y(:, c(1));
            x2 = Y(:, c(2));
            
            d = x1 - x2;
            t_stat = mean(d) / (std(d) / sqrt(n));
            df_t = n - 1;
            p_raw = 2 * (1 - tcdf(abs(t_stat), df_t));
            p_bonf = min(p_raw * n_comp, 1);
            
            if p_bonf < 0.001, ps = '***';
            elseif p_bonf < 0.01, ps = '**';
            elseif p_bonf < 0.05, ps = '*';
            else, ps = 'n.s.'; end
            
            fprintf('  %s: diff=%.4f, t(%d)=%.2f, p=%.4f %s\n', ...
                comp_names{iC}, mean(d), df_t, t_stat, p_bonf, ps);
        end
    end
end
%% Figure 8) Model Comparison by datasize
clc; close all;

% =========================================================================
% [0. 기본 설정]
% =========================================================================
base_dir = "D:\바탕화면\데이터\Arm_Swing_Paper";
models       = ["CNN", "Transformer", "Physres_no_physics"];
model_labels = ["CNN", "Transformer", "Proposed"];

% ============================================================
%  [사용자 설정]
% ============================================================
COLOR_OPTION   = 'A';
SHOW_XLABEL    = false;
SHOW_YLABEL    = false;
use_real_scale = false;

% --- Y-Limit / Y-Tick ---
ylim_nrmse_standard  = [0.04, 0.10]; ytick_nrmse_standard = 0.04:0.02:0.10;
ylim_corr_standard   = [0.92, 0.98]; ytick_corr_standard  = 0.92:0.02:0.98;
ylim_nrmse_speed     = [0.10, 0.16]; ytick_nrmse_speed    = 0.10:0.02:0.16;
ylim_corr_speed      = [0.80, 0.95]; ytick_corr_speed     = 0.80:0.05:0.95;

% ============================================================
%  색상 옵션
% ============================================================
switch COLOR_OPTION
    case 'A'
        model_colors = [ ...
            0.7529  0.2235  0.1686; ...
            0.1529  0.6824  0.3765; ...
            0.0000  0.3137  0.6196; ];
    case 'B'
        model_colors = [ ...
            0.8784  0.4824  0.2235; ...
            0.4824  0.3686  0.6549; ...
            0.0000  0.3137  0.6196; ];
    case 'C'
        model_colors = [ ...
            0.4980  0.5490  0.5529; ...
            0.1020  0.5647  0.5647; ...
            0.0000  0.3137  0.6196; ];
end
model_markers = ['o', 'o', 'o'];

% ============================================================
%  Figure / 스타일 설정
% ============================================================
FONT_NAME      = 'Helvetica';
FONT_SIZE      = 10;
linewidth_axis = 1.1;
linewidth_line = 1.7;
MARKER_SZ      = 4.5;
CAP_SIZE       = 0;
fig_size       = [300 300 240 210];
AX_POS         = [0.22 0.18 0.74 0.76];

% ============================================================
%  분석 루프
% ============================================================
metrics_to_plot = ["nRMSE", "Correlation"];

for met_idx = 1:numel(metrics_to_plot)
    current_metric = metrics_to_plot(met_idx);

    fprintf('\n=======================================================\n');
    fprintf(' Processing Metric: %s\n', current_metric);
    fprintf('=======================================================\n');

    x_values        = [5, 10, 20, 50, 100];
    x_labels_common = ["5", "10", "20", "50", "100"];

    if use_real_scale
        x_axis = x_values;
        x_lim  = [0, 105];
    else
        x_axis = 1:numel(x_values);
        x_lim  = [0.5, 5.5];
    end

    % Config 정의
    configA.id = "A"; configA.mode = "standard";
    configA.folders = [ ...
        "Figure5_datasize", "5percent";
        "Figure5_datasize", "10percent";
        "Figure5_datasize", "20percent";
        "Figure5_datasize", "50percent";
        "Figure3_loso",     ""          ];

    configC.id = "C"; configC.mode = "speed";
    configC.folders = [ ...
        "Figure6_datasize_sploso", "5percent";
        "Figure6_datasize_sploso", "10percent";
        "Figure6_datasize_sploso", "20percent";
        "Figure6_datasize_sploso", "50percent";
        "Figure4_speedloso",       ""          ];

    dataA = loadFigData(configA, models, base_dir, current_metric);
    dataC = loadFigData(configC, models, base_dir, current_metric);

    % Y 범위 선택
    if current_metric == "nRMSE"
        y_label_text = 'nRMSE';
        ylim_A = ylim_nrmse_standard; ytick_A = ytick_nrmse_standard;
        ylim_C = ylim_nrmse_speed;    ytick_C = ytick_nrmse_speed;
    else
        y_label_text = 'Correlation (R)';
        ylim_A = ylim_corr_standard;  ytick_A = ytick_corr_standard;
        ylim_C = ylim_corr_speed;     ytick_C = ytick_corr_speed;
    end

    apply_style = @(ax, yt, xl) set(ax, ...
        'FontName',  FONT_NAME, ...
        'FontSize',  FONT_SIZE, ...
        'Box',       'off', ...
        'TickDir',   'out', ...
        'LineWidth', linewidth_axis, ...
        'XLim',      xl, ...
        'YTick',     yt);

    n_models = numel(models);

    % ------------------------------------------------------------------
    % [Figure 1] Standard LOSO
    % ------------------------------------------------------------------
    figure('Color', 'w', 'Units', 'pixels', ...
        'Position', fig_size, ...
        'Name', sprintf('Standard LOSO [%s]', current_metric));
    hold on; box off;

    for m = 1:n_models
        plot(x_axis, dataA(m, :), ...
            'LineStyle',       '-', ...
            'LineWidth',       linewidth_line, ...
            'Marker',          model_markers(m), ...
            'MarkerSize',      MARKER_SZ, ...
            'Color',           model_colors(m, :), ...
            'MarkerFaceColor', model_colors(m, :), ...
            'MarkerEdgeColor', model_colors(m, :));
    end

    if use_real_scale
        set(gca, 'XTick', x_values);
    else
        set(gca, 'XTick', x_axis, 'XTickLabel', x_labels_common);
    end

    apply_style(gca, ytick_A, x_lim);
    ylim(ylim_A);
    if SHOW_XLABEL
        xlabel('Dataset size (%)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE);
    end
    if SHOW_YLABEL
        ylabel(y_label_text, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE);
    end
    ax = gca; ax.Position = AX_POS;
    hold off;

    % ------------------------------------------------------------------
    % [Figure 2] Speed LOSO
    % ------------------------------------------------------------------
    figure('Color', 'w', 'Units', 'pixels', ...
        'Position', fig_size, ...
        'Name', sprintf('Speed LOSO [%s]', current_metric));
    hold on; box off;

    for m = 1:n_models
        plot(x_axis, dataC(m, :), ...
            'LineStyle',       '-', ...
            'LineWidth',       linewidth_line, ...
            'Marker',          model_markers(m), ...
            'MarkerSize',      MARKER_SZ, ...
            'Color',           model_colors(m, :), ...
            'MarkerFaceColor', model_colors(m, :), ...
            'MarkerEdgeColor', model_colors(m, :));
    end

    if use_real_scale
        set(gca, 'XTick', x_values);
    else
        set(gca, 'XTick', x_axis, 'XTickLabel', x_labels_common);
    end

    apply_style(gca, ytick_C, x_lim);
    ylim(ylim_C);
    if SHOW_XLABEL
        xlabel('Dataset size (%)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE);
    end
    if SHOW_YLABEL
        ylabel(y_label_text, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE);
    end
    ax = gca; ax.Position = AX_POS;
    hold off;

    % ------------------------------------------------------------------
    % 데이터 출력
    % ------------------------------------------------------------------
    fprintf('\n  [%s] Speed LOSO 결과:\n', current_metric);
    fprintf('  %-25s', 'Model');
    for x = 1:numel(x_labels_common)
        fprintf('%10s%%', x_labels_common(x));
    end
    fprintf('\n  %s\n', repmat('-', 1, 75));
    for m = 1:numel(models)
        fprintf('  %-25s', model_labels(m));
        for x = 1:numel(x_labels_common)
            fprintf('%11.4f', dataC(m, x));
        end
        fprintf('\n');
    end
end

fprintf('\n모든 그래프 작성이 완료되었습니다.\n');

%% Figure 7-1) Datsize OOD 컨디션에서 통계검정 
% errorTable에서 피험자별 Mean_nRMSE_all, Mean_Corr_all 추출
clc; close all;

% =========================================================================
% [0. 기본 설정]
% =========================================================================
base_dir = "D:\바탕화면\데이터\Arm_Swing_Paper";
models       = ["CNN", "Transformer", "Physres_no_physics"];
model_labels = ["CNN", "Transformer", "Proposed (BINN)"];
n_models     = numel(models);

x_values = [5, 10, 20, 50, 100];
x_labels = ["5%", "10%", "20%", "50%", "100%"];
n_sizes  = numel(x_values);

% Speed LOSO (OOD) 폴더 구조
size_folders = [ ...
    "Figure6_datasize_sploso", "5percent";
    "Figure6_datasize_sploso", "10percent";
    "Figure6_datasize_sploso", "20percent";
    "Figure6_datasize_sploso", "50percent";
    "Figure4_speedloso",       ""          ];

% 속도 파일
speed_suffixes = ["train_ss1", "train_ss2", "train_ss3"];

% =========================================================================
% [1. errorTable에서 피험자별 데이터 로드]
% =========================================================================
% 먼저 공통 피험자 추출
fprintf('공통 피험자 추출 중...\n');
common_subs = {};

for m = 1:n_models
    for sz = 1:n_sizes
        if size_folders(sz, 2) == ""
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1));
        else
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1), size_folders(sz, 2));
        end
        
        for sf = 1:numel(speed_suffixes)
            fpath = fullfile(data_dir, "errorTable_" + speed_suffixes(sf) + ".mat");
            if ~isfile(fpath), continue; end
            
            S = load(fpath);
            fn = fieldnames(S);
            T = S.(fn{1});
            
            % 피험자 목록 (첫 행 "Total" 제외, S로 시작하는 것만)
            subs = T.RowLabel(startsWith(T.RowLabel, "S"));
            
            if isempty(common_subs)
                common_subs = cellstr(subs);
            else
                common_subs = intersect(common_subs, cellstr(subs));
            end
        end
    end
end
common_subs = sort(common_subs);
n_subs = numel(common_subs);
fprintf('공통 피험자: %d명\n\n', n_subs);

if n_subs == 0
    error('공통 피험자가 없습니다. 경로를 확인하세요.');
end

% 피험자별 nRMSE, Correlation 수집: perf(model, subject, datasize)
perf_nrmse = nan(n_models, n_subs, n_sizes);
perf_corr  = nan(n_models, n_subs, n_sizes);

for m = 1:n_models
    for sz = 1:n_sizes
        if size_folders(sz, 2) == ""
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1));
        else
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1), size_folders(sz, 2));
        end
        
        % 각 피험자의 3속도 평균
        nrmse_by_speed = nan(n_subs, numel(speed_suffixes));
        corr_by_speed  = nan(n_subs, numel(speed_suffixes));
        
        for sf = 1:numel(speed_suffixes)
            fpath = fullfile(data_dir, "errorTable_" + speed_suffixes(sf) + ".mat");
            if ~isfile(fpath), continue; end
            
            S = load(fpath);
            fn = fieldnames(S);
            T = S.(fn{1});
            
            for si = 1:n_subs
                row_idx = find(T.RowLabel == string(common_subs{si}));
                if ~isempty(row_idx)
                    nrmse_by_speed(si, sf) = T.Mean_nRMSE_all(row_idx);
                    corr_by_speed(si, sf)  = T.Mean_Corr_all(row_idx);
                end
            end
        end
        
        % 3속도 평균
        perf_nrmse(m, :, sz) = mean(nrmse_by_speed, 2, 'omitnan');
        perf_corr(m, :, sz)  = mean(corr_by_speed, 2, 'omitnan');
    end
    fprintf('  %s 로드 완료\n', models(m));
end

% =========================================================================
% [2. 데이터 확인 출력]
% =========================================================================
fprintf('\n  [확인] 모델별 평균 성능 (OOD, 속도 평균):\n');
for met_idx = 1:2
    if met_idx == 1
        perf = perf_nrmse; mname = 'nRMSE';
    else
        perf = perf_corr; mname = 'Correlation';
    end
    fprintf('\n  --- %s ---\n', mname);
    fprintf('  %-25s', 'Model');
    for sz = 1:n_sizes, fprintf('%9s', x_labels(sz)); end
    fprintf('\n');
    for m = 1:n_models
        fprintf('  %-25s', model_labels(m));
        for sz = 1:n_sizes
            fprintf('%9.4f', mean(squeeze(perf(m,:,sz)), 'omitnan'));
        end
        fprintf('\n');
    end
end

% =========================================================================
% [3. 통계 검정]
% =========================================================================
binn_idx = find(models == "Physres_no_physics");
baseline_indices = setdiff(1:n_models, binn_idx);

metric_names = ["nRMSE", "Correlation"];
perf_all = {perf_nrmse, perf_corr};

for met_idx = 1:2
    current_metric = metric_names(met_idx);
    perf = perf_all{met_idx};
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════╗\n');
    fprintf('║  Statistical Test: %s (OOD condition)                   \n', current_metric);
    fprintf('╚══════════════════════════════════════════════════════════╝\n');
    
    for bi = baseline_indices
        baseline_name = model_labels(bi);
        
        fprintf('\n  ────────────────────────────────────────────────\n');
        fprintf('  BINN vs %s\n', baseline_name);
        fprintf('  ────────────────────────────────────────────────\n');
        
        % 격차 계산: gap(subject, datasize)
        gap = nan(n_subs, n_sizes);
        for sz = 1:n_sizes
            if current_metric == "nRMSE"
                % nRMSE: baseline - BINN (양수 = BINN이 좋음)
                gap(:, sz) = squeeze(perf(bi, :, sz) - perf(binn_idx, :, sz))';
            else
                % Correlation: BINN - baseline (양수 = BINN이 좋음)
                gap(:, sz) = squeeze(perf(binn_idx, :, sz) - perf(bi, :, sz))';
            end
        end
        
        % 격차 출력
        fprintf('\n  Data size:      ');
        for sz = 1:n_sizes, fprintf('%9s', x_labels(sz)); end
        fprintf('\n  Mean gap:       ');
        for sz = 1:n_sizes, fprintf('%9.4f', mean(gap(:,sz),'omitnan')); end
        fprintf('\n  SD gap:         ');
        for sz = 1:n_sizes, fprintf('%9.4f', std(gap(:,sz),'omitnan')); end
        fprintf('\n');
        
        % -----------------------------------------------------------------
        % [Method 1] Spearman: 격차 vs 데이터 크기
        % -----------------------------------------------------------------
        mean_gaps = mean(gap, 1, 'omitnan');
        [rho_group, p_group] = corr(x_values', mean_gaps', 'Type', 'Spearman');
        
        % 피험자별 Spearman
        rho_list = nan(n_subs, 1);
        for s = 1:n_subs
            sub_gap = gap(s, :);
            if sum(~isnan(sub_gap)) >= 3
                [r, ~] = corr(x_values', sub_gap', 'Type', 'Spearman');
                rho_list(s) = r;
            end
        end
        
        fprintf('\n  [Method 1] Spearman (gap vs data size)\n');
        fprintf('  Group-level:   rho = %.4f, p = %.4f', rho_group, p_group);
        if p_group < 0.01, fprintf(' (**)');
        elseif p_group < 0.05, fprintf(' (*)');
        else, fprintf(' (n.s.)'); end
        fprintf('\n');
        fprintf('  Subject-mean:  rho = %.4f +/- %.4f\n', ...
            mean(rho_list,'omitnan'), std(rho_list,'omitnan'));
        if rho_group < 0
            fprintf('  -> 데이터 축소 시 BINN 우위 확대\n');
        else
            fprintf('  -> 격차 확대 경향 미확인\n');
        end
        
        % -----------------------------------------------------------------
        % [Method 2] Paired t-test: 5% 격차 vs 100% 격차
        % -----------------------------------------------------------------
        gap_5   = gap(:, 1);
        gap_100 = gap(:, end);
        [~, p_tt, ~, stats_tt] = ttest(gap_5, gap_100);
        
        fprintf('\n  [Method 2] Paired t-test (5%% gap vs 100%% gap)\n');
        fprintf('  5%%  gap:  %.4f +/- %.4f\n', mean(gap_5,'omitnan'), std(gap_5,'omitnan'));
        fprintf('  100%% gap: %.4f +/- %.4f\n', mean(gap_100,'omitnan'), std(gap_100,'omitnan'));
        fprintf('  t = %.3f, p = %.4f', stats_tt.tstat, p_tt);
        if p_tt < 0.01, fprintf(' (**)');
        elseif p_tt < 0.05, fprintf(' (*)');
        else, fprintf(' (n.s.)'); end
        fprintf('\n');
        if mean(gap_5,'omitnan') > mean(gap_100,'omitnan')
            fprintf('  -> 5%% 조건에서 격차가 더 큼\n');
        else
            fprintf('  -> 100%% 조건에서 격차가 더 큼\n');
        end
        
        % -----------------------------------------------------------------
        % [Method 3] 각 데이터 크기에서 BINN vs Baseline paired t-test
        % -----------------------------------------------------------------
        fprintf('\n  [Method 3] Per-datasize paired t-test\n');
        fprintf('  %8s | %10s | %10s | %8s | %8s\n', ...
            'Size', 'BINN', char(baseline_name), 't-stat', 'p-val');
        fprintf('  %s\n', repmat('-', 1, 58));
        
        for sz = 1:n_sizes
            binn_vals = squeeze(perf(binn_idx, :, sz))';
            base_vals = squeeze(perf(bi, :, sz))';
            
            valid = ~isnan(binn_vals) & ~isnan(base_vals);
            if sum(valid) < 3
                fprintf('  %8s | insufficient data\n', x_labels(sz));
                continue;
            end
            
            [~, p_sz, ~, stats_sz] = ttest(binn_vals(valid), base_vals(valid));
            
            sig = 'n.s.';
            if p_sz < 0.01, sig = '**';
            elseif p_sz < 0.05, sig = '*'; end
            
            fprintf('  %8s | %10.4f | %10.4f | %8.3f | %.4f %s\n', ...
                x_labels(sz), ...
                mean(binn_vals(valid)), mean(base_vals(valid)), ...
                stats_sz.tstat, p_sz, sig);
        end
    end
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  통계 검정 완료                                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
%% Figure 10) Ablation - Speed LOSO Only (Figure 9 스타일 적용)
clc; close all;
% =========================================================================
% [0. 기본 설정]
% =========================================================================
base_dir = "D:\바탕화면\데이터\Arm_Swing_Paper\Ablation"; 
models = ["CNN_with_model", "no_attention", "Physres_no_zero_init","Physres_no_physics","Full"]; 
model_labels = ["CNN_with_model", "no_attention", "Physres_no_zero_init", "Proposed","Full"]; 

% ============================================================
%  [사용자 설정]
% ============================================================
SHOW_XLABEL    = false;
SHOW_YLABEL    = false;
use_real_scale = false;

% =========================================================================
% [★ 중요 ★] Y-Limit 및 Y-Tick 설정 (Speed LOSO) — 원본 유지
% =========================================================================
ylim_nrmse_speed  = [0.10, 0.15]; 
ytick_nrmse_speed = 0.10 : 0.01 : 0.15; 
ylim_corr_speed   = [0.85, 0.95];
ytick_corr_speed  = 0.85 : 0.02 : 0.95;

% ============================================================
%  색상 설정: Proposed = Figure 9 파랑, 나머지 구분 색상
% ============================================================
model_colors = [ ...
    0.8500  0.3250  0.0980;     % CNN_with_model (주황)
    0.9290  0.6940  0.1250; ...  % No attention (노랑)
    80/255  166/255 196/255; ...  % Physres_no_zero_init (하늘)
    0.0000  0.3137  0.6196; ...  % Proposed (파랑 — Figure 9 동일)
    155/255  187/255 89/255; ...  % Full (초록)
];
model_markers = ['o', 'o', 'o', 'o', 'o'];

% ============================================================
%  Figure / 스타일 설정 (Figure 9 동일)
% ============================================================
FONT_NAME      = 'Helvetica';
FONT_SIZE      = 10;
linewidth_axis = 1.1;
linewidth_line = 1.7;
MARKER_SZ      = 4.5;
fig_size       = [300 300 350 300];
AX_POS         = [0.22 0.18 0.74 0.76];

apply_style = @(ax, yt, xl) set(ax, ...
    'FontName',  FONT_NAME, ...
    'FontSize',  FONT_SIZE, ...
    'Box',       'off', ...
    'TickDir',   'out', ...
    'LineWidth', linewidth_axis, ...
    'XLim',      xl, ...
    'YTick',     yt);

% ============================================================
%  분석 루프
% ============================================================
metrics_to_plot = ["nRMSE", "Correlation"];
x_values        = [5, 10, 20, 50, 100];
x_labels_common = ["5", "10", "20", "50", "100"];

if use_real_scale
    x_axis = x_values;
    x_lim  = [0, 105];
else
    x_axis = 1:numel(x_values);
    x_lim  = [0.5, 5.5];
end

for met_idx = 1:numel(metrics_to_plot)
    current_metric = metrics_to_plot(met_idx);
    
    fprintf('\n=======================================================\n');
    fprintf(' Processing Metric: %s\n', current_metric);
    fprintf('=======================================================\n');
    
    % -----------------------------------------------------------------
    % Config 정의 (Speed LOSO Only)
    % -----------------------------------------------------------------
    configC.id = "C"; configC.mode = "speed";
    configC.folders = [ ...
        "Figure6_datasize_sploso", "5percent"; 
        "Figure6_datasize_sploso", "10percent"; 
        "Figure6_datasize_sploso", "20percent"; 
        "Figure6_datasize_sploso", "50percent";
        "Figure4_speedloso", ""
    ];
    
    dataC = loadFigData(configC, models, base_dir, current_metric); 
    
    % -----------------------------------------------------------------
    % Y 범위 선택
    % -----------------------------------------------------------------
    if current_metric == "nRMSE"
        y_label_text = 'nRMSE';
        curr_ylim  = ylim_nrmse_speed;
        curr_ytick = ytick_nrmse_speed;
    else
        y_label_text = 'Correlation (R)';
        curr_ylim  = ylim_corr_speed;
        curr_ytick = ytick_corr_speed;
    end
    
    % -----------------------------------------------------------------
    % 그래프 그리기 (Figure 9 스타일)
    % -----------------------------------------------------------------
    figure('Color', 'w', 'Units', 'pixels', ...
        'Position', fig_size, ...
        'Name', sprintf('Ablation Speed LOSO [%s]', current_metric));
    hold on; box off;
    
    n_models = numel(models);
    
    for m = 1:n_models
        plot(x_axis, dataC(m, :), ...
            'LineStyle',       '-', ...
            'LineWidth',       linewidth_line, ...
            'Marker',          model_markers(m), ...
            'MarkerSize',      MARKER_SZ, ...
            'Color',           model_colors(m, :), ...
            'MarkerFaceColor', model_colors(m, :), ...
            'MarkerEdgeColor', model_colors(m, :));
    end
    
    % --- 축 스타일 ---
    if use_real_scale
        set(gca, 'XTick', x_values);
    else
        set(gca, 'XTick', x_axis, 'XTickLabel', x_labels_common);
    end
    
    apply_style(gca, curr_ytick, x_lim);
    ylim(curr_ylim);
    
    if SHOW_XLABEL
        xlabel('Dataset size (%)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE);
    end
    if SHOW_YLABEL
        ylabel(y_label_text, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE);
    end
    
    ax = gca; ax.Position = AX_POS;
    hold off;
    
    % -----------------------------------------------------------------
    % 데이터 출력
    % -----------------------------------------------------------------
    fprintf('\n  [%s] Speed LOSO 결과:\n', current_metric);
    fprintf('  %-25s', 'Model');
    for x = 1:numel(x_labels_common)
        fprintf('%10s%%', x_labels_common(x));
    end
    fprintf('\n  %s\n', repmat('-', 1, 75));
    for m = 1:n_models
        fprintf('  %-25s', model_labels(m));
        for x = 1:numel(x_labels_common)
            fprintf('%11.4f', dataC(m, x));
        end
        fprintf('\n');
    end
end

fprintf('\n모든 그래프 작성이 완료되었습니다.\n');

%% Figure 9-1) Statistical Test: Ablation Study (OOD condition)
% Proposed(BINN) vs 각 ablation variant 성능 비교
clc; close all;

% =========================================================================
% [0. 기본 설정]
% =========================================================================
base_dir = "D:\바탕화면\데이터\Arm_Swing_Paper\Ablation";
models       = ["CNN_with_model", "no_attention", "Physres_no_zero_init", "Physres_no_physics", "Full"];
model_labels = ["CNN_{IMU+Phys}", "w/o Cross-Attn", "w/o Zero-Init", "Proposed (BINN)", "w/ Hard Residual"];
n_models     = numel(models);

% Proposed index
proposed_idx = find(models == "Physres_no_physics");

x_values = [5, 10, 20, 50, 100];
x_labels = ["5%", "10%", "20%", "50%", "100%"];
n_sizes  = numel(x_values);

% Speed LOSO (OOD) 폴더 구조
size_folders = [ ...
    "Figure6_datasize_sploso", "5percent";
    "Figure6_datasize_sploso", "10percent";
    "Figure6_datasize_sploso", "20percent";
    "Figure6_datasize_sploso", "50percent";
    "Figure4_speedloso",       ""          ];

speed_suffixes = ["train_ss1", "train_ss2", "train_ss3"];

% =========================================================================
% [1. 공통 피험자 추출]
% =========================================================================
fprintf('공통 피험자 추출 중...\n');
common_subs = {};

for m = 1:n_models
    for sz = 1:n_sizes
        if size_folders(sz, 2) == ""
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1));
        else
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1), size_folders(sz, 2));
        end
        
        for sf = 1:numel(speed_suffixes)
            fpath = fullfile(data_dir, "errorTable_" + speed_suffixes(sf) + ".mat");
            if ~isfile(fpath), continue; end
            
            S = load(fpath);
            fn = fieldnames(S);
            T = S.(fn{1});
            subs = T.RowLabel(startsWith(T.RowLabel, "S"));
            
            if isempty(common_subs)
                common_subs = cellstr(subs);
            else
                common_subs = intersect(common_subs, cellstr(subs));
            end
        end
    end
end
common_subs = sort(common_subs);
n_subs = numel(common_subs);
fprintf('공통 피험자: %d명\n\n', n_subs);

if n_subs == 0
    error('공통 피험자가 없습니다. 경로를 확인하세요.');
end

% =========================================================================
% [2. 피험자별 데이터 로드]
% =========================================================================
perf_nrmse = nan(n_models, n_subs, n_sizes);
perf_corr  = nan(n_models, n_subs, n_sizes);

for m = 1:n_models
    for sz = 1:n_sizes
        if size_folders(sz, 2) == ""
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1));
        else
            data_dir = fullfile(base_dir, models(m), size_folders(sz, 1), size_folders(sz, 2));
        end
        
        nrmse_by_speed = nan(n_subs, numel(speed_suffixes));
        corr_by_speed  = nan(n_subs, numel(speed_suffixes));
        
        for sf = 1:numel(speed_suffixes)
            fpath = fullfile(data_dir, "errorTable_" + speed_suffixes(sf) + ".mat");
            if ~isfile(fpath), continue; end
            
            S = load(fpath);
            fn = fieldnames(S);
            T = S.(fn{1});
            
            for si = 1:n_subs
                row_idx = find(T.RowLabel == string(common_subs{si}));
                if ~isempty(row_idx)
                    nrmse_by_speed(si, sf) = T.Mean_nRMSE_all(row_idx);
                    corr_by_speed(si, sf)  = T.Mean_Corr_all(row_idx);
                end
            end
        end
        
        perf_nrmse(m, :, sz) = mean(nrmse_by_speed, 2, 'omitnan');
        perf_corr(m, :, sz)  = mean(corr_by_speed, 2, 'omitnan');
    end
    fprintf('  %s 로드 완료\n', models(m));
end

% =========================================================================
% [3. 데이터 확인]
% =========================================================================
fprintf('\n  [확인] 모델별 평균 성능 (OOD, 속도 평균):\n');
for met_idx = 1:2
    if met_idx == 1, perf = perf_nrmse; mname = 'nRMSE';
    else, perf = perf_corr; mname = 'Correlation'; end
    fprintf('\n  --- %s ---\n', mname);
    fprintf('  %-25s', 'Model');
    for sz = 1:n_sizes, fprintf('%9s', x_labels(sz)); end
    fprintf('\n');
    for m = 1:n_models
        fprintf('  %-25s', model_labels(m));
        for sz = 1:n_sizes
            fprintf('%9.4f', mean(squeeze(perf(m,:,sz)), 'omitnan'));
        end
        fprintf('\n');
    end
end

% =========================================================================
% [4. 통계 검정: Proposed vs 각 ablation variant]
% =========================================================================
ablation_indices = setdiff(1:n_models, proposed_idx);
metric_names = ["nRMSE", "Correlation"];
perf_all = {perf_nrmse, perf_corr};

for met_idx = 1:2
    current_metric = metric_names(met_idx);
    perf = perf_all{met_idx};
    
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════╗\n');
    fprintf('║  Ablation Statistical Test: %s (OOD)                    \n', current_metric);
    fprintf('╚══════════════════════════════════════════════════════════╝\n');
    
    for ai = ablation_indices
        abl_name = model_labels(ai);
        
        fprintf('\n  ────────────────────────────────────────────────\n');
        fprintf('  Proposed vs %s\n', abl_name);
        fprintf('  ────────────────────────────────────────────────\n');
        
        % 격차: gap(subject, datasize)
        gap = nan(n_subs, n_sizes);
        for sz = 1:n_sizes
            if current_metric == "nRMSE"
                % nRMSE: ablation - proposed (양수 = proposed가 좋음)
                gap(:, sz) = squeeze(perf(ai, :, sz) - perf(proposed_idx, :, sz))';
            else
                % Corr: proposed - ablation (양수 = proposed가 좋음)
                gap(:, sz) = squeeze(perf(proposed_idx, :, sz) - perf(ai, :, sz))';
            end
        end
        
        % 격차 출력
        fprintf('\n  Data size:      ');
        for sz = 1:n_sizes, fprintf('%9s', x_labels(sz)); end
        fprintf('\n  Mean gap:       ');
        for sz = 1:n_sizes, fprintf('%9.4f', mean(gap(:,sz),'omitnan')); end
        fprintf('\n  SD gap:         ');
        for sz = 1:n_sizes, fprintf('%9.4f', std(gap(:,sz),'omitnan')); end
        fprintf('\n');
        
        % -----------------------------------------------------------------
        % [Method 1] Spearman: 격차 vs 데이터 크기
        % -----------------------------------------------------------------
        mean_gaps = mean(gap, 1, 'omitnan');
        [rho_group, p_group] = corr(x_values', mean_gaps', 'Type', 'Spearman');
        
        rho_list = nan(n_subs, 1);
        for s = 1:n_subs
            sub_gap = gap(s, :);
            if sum(~isnan(sub_gap)) >= 3
                [r, ~] = corr(x_values', sub_gap', 'Type', 'Spearman');
                rho_list(s) = r;
            end
        end
        
        fprintf('\n  [Method 1] Spearman (gap vs data size)\n');
        fprintf('  Group-level:   rho = %.4f, p = %.4f', rho_group, p_group);
        if p_group < 0.01, fprintf(' (**)');
        elseif p_group < 0.05, fprintf(' (*)');
        else, fprintf(' (n.s.)'); end
        fprintf('\n');
        fprintf('  Subject-mean:  rho = %.4f +/- %.4f\n', ...
            mean(rho_list,'omitnan'), std(rho_list,'omitnan'));
        
        % -----------------------------------------------------------------
        % [Method 2] Paired t-test: 5% gap vs 100% gap
        % -----------------------------------------------------------------
        gap_5   = gap(:, 1);
        gap_100 = gap(:, end);
        [~, p_tt, ~, stats_tt] = ttest(gap_5, gap_100);
        
        fprintf('\n  [Method 2] Paired t-test (5%% gap vs 100%% gap)\n');
        fprintf('  5%%  gap:  %.4f +/- %.4f\n', mean(gap_5,'omitnan'), std(gap_5,'omitnan'));
        fprintf('  100%% gap: %.4f +/- %.4f\n', mean(gap_100,'omitnan'), std(gap_100,'omitnan'));
        fprintf('  t = %.3f, p = %.4f', stats_tt.tstat, p_tt);
        if p_tt < 0.01, fprintf(' (**)');
        elseif p_tt < 0.05, fprintf(' (*)');
        else, fprintf(' (n.s.)'); end
        fprintf('\n');
        
        % -----------------------------------------------------------------
        % [Method 3] Per-datasize paired t-test
        % -----------------------------------------------------------------
        fprintf('\n  [Method 3] Per-datasize paired t-test\n');
        fprintf('  %8s | %10s | %10s | %8s | %8s\n', ...
            'Size', 'Proposed', char(abl_name), 't-stat', 'p-val');
        fprintf('  %s\n', repmat('-', 1, 58));
        
        for sz = 1:n_sizes
            prop_vals = squeeze(perf(proposed_idx, :, sz))';
            abl_vals  = squeeze(perf(ai, :, sz))';
            
            valid = ~isnan(prop_vals) & ~isnan(abl_vals);
            if sum(valid) < 3
                fprintf('  %8s | insufficient data\n', x_labels(sz));
                continue;
            end
            
            [~, p_sz, ~, stats_sz] = ttest(prop_vals(valid), abl_vals(valid));
            
            sig = 'n.s.';
            if p_sz < 0.01, sig = '**';
            elseif p_sz < 0.05, sig = '*'; end
            
            % 방향 표시
            if current_metric == "nRMSE"
                better = '';
                if mean(prop_vals(valid)) < mean(abl_vals(valid)), better = ' [P]';
                else, better = ' [A]'; end
            else
                better = '';
                if mean(prop_vals(valid)) > mean(abl_vals(valid)), better = ' [P]';
                else, better = ' [A]'; end
            end
            
            fprintf('  %8s | %10.4f | %10.4f | %8.3f | %.4f %s%s\n', ...
                x_labels(sz), ...
                mean(prop_vals(valid)), mean(abl_vals(valid)), ...
                stats_sz.tstat, p_sz, sig, better);
        end
        fprintf('  [P] = Proposed 우위, [A] = Ablation 우위\n');
    end
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  Ablation 통계 검정 완료                                  ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');
%% Figure 5-2) Model Comparison by datasize in one figure
% 1. Standard LOSO는 '실선(-)', Speed LOSO는 '점선(--)'으로 표현합니다.
% 2. X축은 공유하고, Y축 범위를 두 데이터가 모두 들어오도록 확장합니다.
% 3. 함수 중복 정의 문제를 해결했습니다.
clc; close all;

% =========================================================================
% [0. 기본 설정]
% =========================================================================
base_dir = "D:\바탕화면\데이터\Arm_Swing_Paper"; 
models = [ "Physres_no_physics"]; 
model_labels = [ "Proposed"]; 
model_colors = [ ...
    0.8500 0.3250 0.0980; ... % CNN (주황)
    0.9290 0.6940 0.1250; ... % Transformer (노랑)
    0.0000 0.4470 0.7410; ... % PhysResNet (파랑)
];
model_markers = ["s", "^", "o"];

% =========================================================================
% [★ 중요 ★] Y-Limit 및 Y-Tick 설정 (통합 범위)
% =========================================================================
% Standard(A)와 Speed(C)의 범위가 다르므로, 둘 다 포함하는 '최대 범위'로 설정

% --- 1. nRMSE 설정 ---
% A(0.06~0.12)와 C(0.11~0.17)를 모두 포함하려면 0.06 ~ 0.18 정도가 적당
ylim_nrmse_merged  = [0.06, 0.18]; 
ytick_nrmse_merged = 0.06 : 0.02 : 0.18; 

% --- 2. Correlation 설정 ---
% A(0.90~0.96)와 C(0.80~0.95)를 모두 포함하려면 0.80 ~ 1.00
ylim_corr_merged   = [0.80, 1.00];
ytick_corr_merged  = 0.80 : 0.05 : 1.00;

% =========================================================================
% [1. 분석 루프 실행]
% =========================================================================
metrics_to_plot = ["nRMSE", "Correlation"];

for met_idx = 1:numel(metrics_to_plot)
    current_metric = metrics_to_plot(met_idx);
    
    fprintf('\n=======================================================\n');
    fprintf(' Processing Metric: %s\n', current_metric);
    fprintf('=======================================================\n');
    
    % ---------------------------------------------------------------------
    % 설정 정의 (A: Standard / C: Speed)
    % ---------------------------------------------------------------------
    x_labels_common = ["5", "10", "20", "50", "100"];
    
    % Config A: Standard
    configA.id = "A"; configA.title = "Standard LOSO";
    configA.x_labels = x_labels_common; configA.mode = "standard";
    configA.folders = [ ...
        "Figure5_datasize", "5percent"; 
        "Figure5_datasize", "10percent"; 
        "Figure5_datasize", "20percent"; 
        "Figure5_datasize", "50percent";
        "Figure3_loso", "" 

    ];
    
    % Config C: Speed
    configC.id = "C"; configC.title = "Speed LOSO";
    configC.x_labels = x_labels_common; configC.mode = "speed";
    configC.folders = [ ...
        "Figure6_datasize_sploso", "5percent"; 
        "Figure6_datasize_sploso", "10percent"; 
        "Figure6_datasize_sploso", "20percent"; 
        "Figure6_datasize_sploso", "50percent";
        "Figure4_speedloso", ""

    ];
    
    % ---------------------------------------------------------------------
    % 데이터 로드
    % ---------------------------------------------------------------------
    dataA = loadFigData(configA, models, base_dir, current_metric); 
    dataC = loadFigData(configC, models, base_dir, current_metric); 
    
    % ---------------------------------------------------------------------
    % 범위 및 라벨 설정
    % ---------------------------------------------------------------------
    if current_metric == "nRMSE"
        y_label_text = "nRMSE";
        curr_ylim = ylim_nrmse_merged;
        curr_ytick = ytick_nrmse_merged;
    else
        y_label_text = "Correlation (R)";
        curr_ylim = ylim_corr_merged;
        curr_ytick = ytick_corr_merged;
    end
    
    % ---------------------------------------------------------------------
    % 그래프 그리기 (Overlay)
    % ---------------------------------------------------------------------
    figure('Color', 'w', 'Position', [100, 100, 330, 280], 'Name', sprintf('Overlay Plot [%s]', current_metric));
    ax = axes();
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    
    n_models = numel(models);
    n_points = numel(x_labels_common);
    x_axis = 1:n_points;
    
    
    legend_handles = [];
    
    for m = 1:n_models
        % 1. Standard LOSO (실선, Solid)
        h1 = plot(ax, x_axis, dataA(m, :), ...
            'LineStyle', '-', 'LineWidth', 1.8, ... 
            'Marker', model_markers(m), 'MarkerSize', 5, ...
            'Color', model_colors(m, :), 'MarkerFaceColor', model_colors(m, :), ...
            'DisplayName', sprintf('%s (Sub)', model_labels(m)));
        
        % 2. Speed LOSO (점선, Dashed)
        h2 = plot(ax, x_axis, dataC(m, :), ...
            'LineStyle', '--', 'LineWidth', 1.8, ... 
            'Marker', model_markers(m), 'MarkerSize', 5, ...
            'Color', model_colors(m, :), 'MarkerFaceColor', 'w', ... % 내부 흰색으로 구분
            'DisplayName', sprintf('%s (Spd)', model_labels(m)));
            
        legend_handles = [legend_handles, h1, h2];
    end
    
    % --- 스타일 꾸미기 ---
    set(ax, 'XTick', x_axis, 'XTickLabel', x_labels_common);
    set(ax, 'YTick', curr_ytick);
    ylim(ax, curr_ylim);
    xlim(ax, [0.5, 5.5]);
    
    set(ax, 'FontSize', 9, 'FontName', 'Helvetica', 'FontWeight', 'bold');
    xlabel(ax, 'Dataset size (%)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel(ax, y_label_text, 'FontSize', 10, 'FontWeight', 'bold');
    
    % 범례 (2열 배치)
    legend(ax, legend_handles, 'Location', 'best', 'FontSize', 7, 'Box', 'on', 'NumColumns', 2);
    
    hold(ax, 'off');
    
end
fprintf('\n모든 그래프 작성이 완료되었습니다.\n');



% ------------------------------------------------------------------------
% 1. 데이터 로딩 함수 (Mean 값만 추출)
function data = loadFigData(figConfig, models, base_dir, metric_type)
    n_models = numel(models);
    n_points = numel(figConfig.x_labels);
    data = NaN(n_models, n_points);
    
    fprintf('  Loading Data for %s...\n', figConfig.title);
    
    for m = 1:n_models
        curr_model = models(m);
        for x = 1:n_points
            main_folder = figConfig.folders(x, 1);
            sub_folder  = figConfig.folders(x, 2);
            target_path = fullfile(base_dir, curr_model, main_folder, sub_folder);
            
            val = NaN;
            if figConfig.mode == "standard"
                [val, ~] = findAndLoad(target_path, "*ss123.mat", metric_type);
            elseif figConfig.mode == "speed"
                vals = [];
                suffixes = ["*ss1.mat", "*ss2.mat", "*ss3.mat"];
                for s = 1:3
                    [v, ~] = findAndLoad(target_path, suffixes(s), metric_type);
                    if ~isnan(v), vals = [vals, v]; end %#ok<AGROW>
                end
                if ~isempty(vals)
                    val = mean(vals);
                end
            end
            data(m, x) = val;
        end
    end
end



%  Local Functions
% ------------------------------------------------------------------------
function processFigureDualMode(figConfig, models, labels, colors, markers, base_dir, metric_type, y_label_text)
    n_models = numel(models);
    n_points = numel(figConfig.x_labels);
    
    mean_vals = NaN(n_models, n_points);
    std_vals  = NaN(n_models, n_points);
    
    % 1. 데이터 로드
    for m = 1:n_models
        curr_model = models(m);
        for x = 1:n_points
            main_folder = figConfig.folders(x, 1);
            sub_folder  = figConfig.folders(x, 2);
            target_path = fullfile(base_dir, curr_model, main_folder, sub_folder);
            
            if figConfig.mode == "standard"
                [val, err] = findAndLoad(target_path, "*ss123.mat", metric_type);
                mean_vals(m, x) = val;
                std_vals(m, x)  = err;
            elseif figConfig.mode == "speed"
                vals = []; errs = [];
                suffixes = ["*ss1.mat", "*ss2.mat", "*ss3.mat"];
                for s = 1:3
                    [v, e] = findAndLoad(target_path, suffixes(s), metric_type);
                    if ~isnan(v)
                        vals = [vals, v]; %#ok<AGROW>
                        errs = [errs, e]; %#ok<AGROW>
                    end
                end
                if ~isempty(vals)
                    mean_vals(m, x) = mean(vals);
                    std_vals(m, x)  = mean(errs);
                end
            end
        end
    end
    
    % 2. 그래프 그리기 (With Std, Mean Only)
    plotTypes = ["WithStd", "MeanOnly"];
    
    for pt = 1:numel(plotTypes)
        pType = plotTypes(pt);
        
        figure('Color', 'w', 'Position', [100 100 230 190]);
        hold on; grid on; box on;
        
        x_axis = 1:n_points;
        
        for m = 1:n_models
            if pType == "WithStd"
                errorbar(x_axis, mean_vals(m, :), std_vals(m, :), ...
                    'LineStyle', '-', 'LineWidth', 1.5, ...
                    'Marker', markers(m), 'MarkerSize', 4, ...
                    'Color', colors(m, :), ...
                    'MarkerFaceColor', colors(m, :), ...
                    'CapSize', 4, ...
                    'DisplayName', labels(m));
            else
                plot(x_axis, mean_vals(m, :), ...
                    'LineStyle', '-', 'LineWidth', 1.7, ... 
                    'Marker', markers(m), 'MarkerSize', 4, ...
                    'Color', colors(m, :), ...
                    'MarkerFaceColor', colors(m, :), ...
                    'DisplayName', labels(m));
            end
        end
        
        % [수정됨] Tick 설정
        set(gca, 'XTick', x_axis, 'XTickLabel', figConfig.x_labels); % X축: 데이터 개수에 맞춤
        
        % [수정됨] Y축 Tick 적용 (설정값이 있는 경우)
        if isfield(figConfig, 'ytick') && ~isempty(figConfig.ytick)
            set(gca, 'YTick', figConfig.ytick);
        end

        % 폰트 설정 (Tick 적용 후 폰트가 리셋될 수 있으므로 마지막에 적용)
        set(gca, 'FontSize', 9, 'FontName', 'Helvetica', 'FontWeight', 'bold');
        
        xlabel(figConfig.x_name, 'FontSize', 9, 'FontWeight', 'bold');
        ylabel(y_label_text, 'FontSize', 9, 'FontWeight', 'bold');
        
        % Y축 범위 적용
        if ~isempty(figConfig.ylim)
            ylim(figConfig.ylim);
        end
        
        legend('Location', 'best', 'FontSize', 8, 'Box', 'off');
        hold off;
    end
end

function [val, err] = findAndLoad(folder_path, file_pattern, metric_type)
    val = NaN; err = NaN;
    search_pattern = fullfile(folder_path, "errorTable_" + file_pattern);
    files = dir(search_pattern);
    
    if isempty(files)
        return;
    end
    
    file_path = fullfile(files(1).folder, files(1).name);
    
    try
        S = load(file_path);
        vars = fieldnames(S);
        if ~isempty(vars)
            T = S.(vars{1}); 
            if istable(T)
                if strcmp(metric_type, "nRMSE")
                    if ismember("Mean_nRMSE_all", T.Properties.VariableNames)
                        val = T.Mean_nRMSE_all(1);
                        err = T.Std_nRMSE_all(1);
                    end
                elseif strcmp(metric_type, "Correlation")
                    if ismember("Mean_Corr_all", T.Properties.VariableNames)
                        val = T.Mean_Corr_all(1);
                        err = T.Std_Corr_all(1);
                    end
                end
            end
        end
    catch
        fprintf('  [Error] Failed to load: %s\n', file_path);
    end
end


% =========================================================================
% [Local Function] 수치값 추출 (포맷팅 없이 숫자만 반환)
% =========================================================================
function [n_mu, n_sd, c_mu, c_sd] = extract_raw_values(fpath)
    % Return format: [Ipsi, Contra, Flex, Ext] (1x4 vector)
    n_mu = [NaN, NaN, NaN, NaN]; n_sd = n_mu;
    c_mu = n_mu; c_sd = n_mu;
    
    if ~isfile(fpath), return; end
    
    try
        S = load(fpath);
        vars = fieldnames(S);
        T = S.(vars{1});
        row_d = T(1, :); 
        
        % 1. nRMSE (x100 적용)
        % Ipsi
        n_mu(1) = (row_d.Mean_nRMSE_Fx_Ipsi + row_d.Mean_nRMSE_Fz_Ipsi) / 2 * 100;
        n_sd(1) = (row_d.Std_nRMSE_Fx_Ipsi + row_d.Std_nRMSE_Fz_Ipsi) / 2 * 100;
        % Contra
        n_mu(2) = (row_d.Mean_nRMSE_Fx_Contra + row_d.Mean_nRMSE_Fz_Contra) / 2 * 100;
        n_sd(2) = (row_d.Std_nRMSE_Fx_Contra + row_d.Std_nRMSE_Fz_Contra) / 2 * 100;
        % Flex
        n_mu(3) = (row_d.Mean_nRMSE_Fx_Flex + row_d.Mean_nRMSE_Fz_Flex) / 2 * 100;
        n_sd(3) = (row_d.Std_nRMSE_Fx_Flex + row_d.Std_nRMSE_Fz_Flex) / 2 * 100;
        % Ext
        n_mu(4) = (row_d.Mean_nRMSE_Fx_Ext + row_d.Mean_nRMSE_Fz_Ext) / 2 * 100;
        n_sd(4) = (row_d.Std_nRMSE_Fx_Ext + row_d.Std_nRMSE_Fz_Ext) / 2 * 100;
        
        % 2. Correlation
        % Ipsi
        c_mu(1) = (row_d.Mean_Corr_Fx_Ipsi + row_d.Mean_Corr_Fz_Ipsi) / 2;
        c_sd(1) = (row_d.Std_Corr_Fx_Ipsi + row_d.Std_Corr_Fz_Ipsi) / 2;
        % Contra
        c_mu(2) = (row_d.Mean_Corr_Fx_Contra + row_d.Mean_Corr_Fz_Contra) / 2;
        c_sd(2) = (row_d.Std_Corr_Fx_Contra + row_d.Std_Corr_Fz_Contra) / 2;
        % Flex
        c_mu(3) = (row_d.Mean_Corr_Fx_Flex + row_d.Mean_Corr_Fz_Flex) / 2;
        c_sd(3) = (row_d.Std_Corr_Fx_Flex + row_d.Std_Corr_Fz_Flex) / 2;
        % Ext
        c_mu(4) = (row_d.Mean_Corr_Fx_Ext + row_d.Mean_Corr_Fz_Ext) / 2;
        c_sd(4) = (row_d.Std_Corr_Fx_Ext + row_d.Std_Corr_Fz_Ext) / 2;
        
    catch
        % Error Handling (Silent or Print)
    end
end

function r = corr_pairwise(x, y)
    valid = isfinite(x) & isfinite(y);
    if sum(valid) < 3
        r = NaN;
        return;
    end
    cc = corrcoef(x(valid), y(valid));
    r = cc(1,2);
end


function plot_mean_sd(ax, ph_pct, xv, mu_t, sd_t, mu_e, sd_e, col_patch_t, col_patch_e, col_line_e)
    patch(ax, xv, [mu_t - sd_t, fliplr(mu_t + sd_t)], col_patch_t, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');
    plot(ax, ph_pct, mu_t, 'k-', 'LineWidth', 2);
    patch(ax, xv, [mu_e - sd_e, fliplr(mu_e + sd_e)], col_patch_e, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.12, 'HandleVisibility', 'off');
    plot(ax, ph_pct, mu_e, '-', 'LineWidth', 2, 'Color', col_line_e);
end