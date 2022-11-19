from dash import Dash, dcc, html, Input, Output, State
import dash_daq as daq
import dash_bootstrap_components as dbc
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from collections import deque

kp = 1
ki = 0
kd = 0
setpoint = 0

app = Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])

t = list(np.linspace(0, 2 * np.pi, 100))
t_interval = np.abs(t[1] - t[0])


app.layout = html.Div([
    html.H3('PID Control Dashboard', style={'textAlign': 'center'}, className='mt-3'),
    dbc.Row([
        dbc.Col([], width=2),
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    dcc.Graph(id='live-graph', animate=True),
                    dcc.Interval(id='interval', interval=100,n_intervals=0),
                    html.Div(children="Current Parameters", id="pid-output", className='mt-3')
                ])
            ])
        ], width=8),
        dbc.Col([], width=2),
    ], class_name="mt-4"),
    dbc.Row([
        dbc.Col([], width=2),
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    "PID Guide"
                ]),
                dbc.CardBody([
                    html.Iframe(src="https://www.youtube.com/embed/UR0hOmjaHp0", width="100%", height="315")
                ])
            ])
        ], width=4),
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    "PID Parameters"
                ]),
                dbc.CardBody([
                    dbc.Row([
                        dbc.Row([
                            dbc.Col([
                                dbc.Label("Proportional Gain", html_for='kp'),
                                dbc.Input(id='kp', type='number', value=kp, min=0)
                            ], width=6),
                            dbc.Col([
                                dbc.Label("Integral Gain", html_for='ki'),
                                dbc.Input(id='ki', type='number', value=ki, min=0)
                            ], width=6)
                        ], className='mb-3'),
                        dbc.Row([
                            dbc.Col([
                                dbc.Label("Derivative Gain", html_for='kd'),
                                dbc.Input(id='kd', type='number', value=kd, min=0)
                            ], width=6),
                            dbc.Col([
                                dbc.Label("Setpoint", html_for='setpoint'),
                                dbc.Input(id='setpoint', type='number', value=setpoint, min=0)
                            ], width=6)
                        ], className='mb-3'),
                        dbc.Row([
                            dbc.Col([
                                daq.PowerButton(id='power-button', on=False, label='OFF', color='#00ff00', labelPosition='bottom')
                            ], width=3),
                            dbc.Col([], width=6),
                            dbc.Col([
                                dbc.Button("Submit", id='pid-submit', color="primary"),
                            ], width=3)
                        ], className='mt-3')
                    ])
                ])
            ])
        ], width=4),
        dbc.Col([], width=2)
    ], class_name="mt-4 mb-4")
])
    
@app.callback(
    Output('live-graph', 'figure'),
    Input('interval', 'n_intervals'))
def live_graph(n_interval):
    t.pop(0)
    t.append(t[-1] + t_interval)
    
    fig = go.Figure()
    
    scatter = go.Scatter(x=t, y=np.sin(t), mode='markers', name='sin')
    scatter2 = go.Scatter(x=t, y=np.cos(t), mode='markers', name='cos')
    scatter3 = go.Scatter(x=t, y=[0.1] * len(t), mode='lines', name='setpoint')
    
    fig.add_traces([scatter, scatter2, scatter3])
    
    fig.layout.xaxis.range = [min(t), max(t)]
    
    fig.layout.showlegend = False
    fig.layout.yaxis.showgrid = False
    fig.layout.xaxis.showgrid = False
    fig.layout.yaxis.zeroline = False
    
    fig.layout.plot_bgcolor = 'rgba(0, 0, 0, 0)'
    fig.layout.paper_bgcolor = 'rgba(0, 0, 0, 0)'
    
    fig.layout.font.color = '#fff'
    fig.layout.title = 'Arduino Output'
    fig.layout.xaxis.title = 'Time (ms)'
    fig.layout.yaxis.title = 'Distance (cm)'
    
    return fig


@app.callback(
    Output('pid-output', 'children'),
    Input('pid-submit', 'n_clicks'),
    State('kp', 'value'),
    State('ki', 'value'),
    State('kd', 'value'),
    State('setpoint', 'value'))
def pid_submit(n_clicks, kp, ki, kd, setpoint):
    print(kp, ki, kd, setpoint)
    return f"Proportional: {kp} Integral: {ki} Derivative: {kd} Setpoint: {setpoint}"

@app.callback(
    Output('power-button', 'label'),
    Input('power-button', 'on'))
def power_button(on):
    if on:
        return 'ON'
    else:
        return 'OFF'


if __name__ == '__main__':
    app.run_server(debug=True)
    
    
    