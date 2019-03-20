close all;
figure;

time_offset = 4;
grid on; hold on;

plot(logElements(idx_loco_contact_schedule_adapted_stride_duration).time-time_offset, ...
     logElements(idx_loco_contact_schedule_adapted_stride_duration).data, ...
     'b','linewidth',2);
   
plot(logElements(idx_loco_contact_schedule_stride_feedback).time-time_offset, ...
     -0.3*logElements(idx_loco_contact_schedule_stride_feedback).data, ...
     '-.r','linewidth',2.5);

   error_norm = sqrt(sum([logElements(idx_loco_torso_linearVelocityErrorInControlFrame_x).data ...
                       logElements(idx_loco_torso_linearVelocityErrorInControlFrame_y).data ...
                       0.35*logElements(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_z).data-...
                       0.35*logElements(idx_loco_torso_measAngularVelocityBaseInControlFrame_z).data ].^2,2));

plot(logElements(idx_loco_contact_schedule_adapted_stride_duration).time-time_offset, ...
  error_norm, 'g','linewidth',2);
   
legend('stride duration','stride feedback','tracking error norm','','');
   
sd_max = plot(logElements(idx_loco_contact_schedule_nominal_stride_duration).time-time_offset, ...
     logElements(idx_loco_contact_schedule_nominal_stride_duration).data, ...
     '--k');

one_vec = ones(length(logElements(idx_loco_contact_schedule_nominal_stride_duration).data),1).';
sd_min = plot(logElements(idx_loco_contact_schedule_nominal_stride_duration).time-time_offset,...
  0.65*one_vec, '--k');
   
xlim([0 15]);

set(get(get(sd_max,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
set(get(get(sd_min,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');




ylim([-0.1 0.9]);
xlabel('time [s]');
ylabel('stride duration [s]');