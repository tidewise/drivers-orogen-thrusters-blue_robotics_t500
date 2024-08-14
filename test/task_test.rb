# frozen_string_literal: true

using_task_library "thrusters_blue_robotics_t500"

describe OroGen.thrusters_blue_robotics_t500.Task do
    run_live

    attr_reader :task
    before do
            @task = syskit_deploy(OroGen.thrusters_blue_robotics_t500.Task
                                        .deployed_as("t500_driver"))

            command_table_path = File.join(__dir__,
                "data/blue_robotics_t500-command_24V.csv")

            @task.properties.command_to_pwm_table_file_path = command_table_path
            @task.properties.no_actuation_pwm_command = 42

            syskit_configure(@task)

    end

    it "raises if cmd_in has different mode than configured" do
        cmd_in = effort_command({ a: 1, b: 2 })
        cmd_in.elements.push(Types.base.JointState.Raw(3))
        cmd_in.names.push("c")

        syskit_start(task)
        expect_execution do
            syskit_write task.cmd_in_port, cmd_in
        end.to do
            emit task.invalid_command_mode_event
            emit task.exception_event
        end
    end

    it "saturates backwards" do
        syskit_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: -10.32 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1100, pwm_out.duty_cycles[0]
    end

    it "saturates forward" do
        syskit_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 16.45 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1900, pwm_out.duty_cycles[0]
    end

    it "sends null pwm command when effort is inside dead zone" do
        syskit_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: -0.159, b: 0.259 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 2
        pwm_out.duty_cycles.each do |x|
            assert_equal task.properties.no_actuation_pwm_command, x
        end
    end

    it "interpolates effort commands" do
        syskit_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 2.21, b: 6.21,
                                          c: 0.69, d: -2.86, e: -6.52 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 5

        expected = [1589, 1695, 1539, 1353, 1229]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    def effort_command(values)
        Types.base.samples.Joints.new(
            time: Time.now,
            names: values.map { |k, _| k.to_s },
            elements: values.map { |_, x| Types.base.JointState.Effort(x) }
        )
    end
end