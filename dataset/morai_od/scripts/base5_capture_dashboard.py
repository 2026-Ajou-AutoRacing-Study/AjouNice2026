#!/usr/bin/env python3
"""Read-only localhost capture dashboard; no ROS dependency or capture controls."""
import argparse
import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path

HTML='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Base5 캡처</title>
<style>body{background:#101824;color:#dbe8f4;font:17px system-ui;margin:40px;max-width:1200px}h1{font-size:28px}progress{width:100%;height:25px}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:12px;border-bottom:1px solid #334} .cards{display:flex;gap:24px;padding:25px 0}pre{white-space:pre-wrap}small{color:#8fa4b8}</style>
<h1>Base5 · 실시간 동기 캡처</h1><small>파일 완결 / 후보 승인 / 검토를 구분한다. 후보 승인은 학습 성능 검증이 아니다.</small>
<p id="stage">연결 중</p><progress id="bar" max="100" value="0"></progress><div id="cards" class="cards"></div>
<h2>패밀리·거리·방향별 진행</h2><table><thead><tr><th>패밀리 / 거리 / 방향</th><th>후보 승인</th><th>검토</th><th>실패</th></tr></thead><tbody id="bins"></tbody></table><pre id="error"></pre>
<script>const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function tick(){try{let s=await(await fetch('/status.json',{cache:'no-store'})).json();
let stale=(Date.now()/1000-s.updated_at)>20&&!['complete','halted','interrupted','stopped','prepared_waiting_for_MR_reconnect'].includes(s.stage);document.getElementById('stage').textContent=`${s.stage} · ${s.current||''} ${stale?'(상태 갱신 지연)':''}`;
bar.max=s.requested||1;bar.value=s.finished;cards.innerHTML=[['완료',`${s.finished}/${s.requested}`],['후보 승인',s.accepted_candidate],['검토',s.review],['실패',s.failed],['예상 잔여',s.finished?`${Math.round(s.eta_sec/60)}분`:'측정 전']].map(([a,b])=>`<div>${esc(a)}<h2>${esc(b)}</h2></div>`).join('');
let bins={};for(let r of s.results||[]){let k=`${r.family} / ${(r.distance_bounds||[]).join('–')}m / ${r.role||''}`;bins[k]??={accepted_candidate:0,review:0,failed:0};bins[k][r.status]++;}
document.getElementById('bins').innerHTML=Object.entries(bins).sort().map(([k,v])=>`<tr><td>${esc(k)}</td><td>${v.accepted_candidate}</td><td>${v.review}</td><td>${v.failed}</td></tr>`).join('');error.textContent=s.error||'';
}catch(e){document.getElementById('stage').textContent='status.json 대기 중'} }tick();setInterval(tick,2000);</script></html>'''


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--port',type=int,default=8075);p.add_argument('--once',action='store_true');a=p.parse_args()
    if a.once:
        print((a.capture_root/'status.json').read_text());return
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path=='/': body=HTML.encode();mime='text/html; charset=utf-8'
            elif self.path=='/status.json':
                try: body=(a.capture_root/'status.json').read_bytes()
                except FileNotFoundError: body=b'{}'
                mime='application/json'
            else: self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
        def log_message(self,*args): pass
    print(f'http://127.0.0.1:{a.port}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()


if __name__=='__main__':main()
